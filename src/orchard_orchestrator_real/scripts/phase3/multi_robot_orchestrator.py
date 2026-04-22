#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
multi_robot_orchestrator.py — 5 车实机采摘-运输编排（动捕直控版）

6 阶段串行：
  S0 gather                : picker→picker_base, transporter→transporter_initial_position
  S1 pickers_to_midpoint   : picker 并行 → picker_midpoint
  S2 transport_midpoint    : Hungarian 分配 2 transporter 服务 3 picker 中的 2 个
                             每个 transporter 走 4 waypoint（east_stop→stop_point→卸货→east_stop）
  S3 pickers_to_endpoint   : picker 并行 → picker_endpoint（transporter 空闲停在 east_stop）
  S4 transport_endpoint    : 复用 S2 配对，在 picker_endpoint 再服务一次（不跑 Hungarian）
  S5 return_to_base        : picker → picker_base（transporter 停在当前 east_stop）

控制层：每 waypoint 为活跃车新建 SimpleController；waypoint 结束 stop() 并弃用。
订阅层：自建 /mocap_node/Robot_N/pose 订阅做 Hungarian 的 pose 输入，
        与 SimpleController 对同一 topic 的订阅互不干扰。
"""

import math
import os
import sys
import threading

try:
    import rospkg
    _pkg_root = rospkg.RosPack().get_path("orchard_orchestrator_real")
    _scripts_dir = os.path.join(_pkg_root, "scripts")
except Exception:
    _scripts_dir = os.path.normpath(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "..")
    )
if _scripts_dir not in sys.path:
    sys.path.insert(0, _scripts_dir)

import rospy
from geometry_msgs.msg import PoseStamped

from controller.simple_controller import SimpleController
from algorithms.cost_matrix import build_cost_matrix
from algorithms.hungarian import assign_tasks, print_assignment_table


def quat_to_yaw(q):
    """四元数 → yaw（内联，不 import tf）。"""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _robot_id_to_int(robot_id):
    """robot7 → 7"""
    return int(robot_id.replace("robot", ""))


class Orchestrator(object):
    # 阶段超时常量（秒）
    TIMEOUT_LANE    = 30.0   # S0/S1/S2/S3/S4（过道段 + 运输段）
    TIMEOUT_RETURN  = 40.0   # S5：返 base
    UNLOAD_DURATION = 5.0    # 卸货 fallback 时长（若 yaml 未给）

    def __init__(self):
        self.differential_mode = bool(rospy.get_param("~differential_mode", True))
        self.skip_gather = bool(rospy.get_param("~skip_gather", False))

        # --- zones.yaml 新 schema ---
        self.field = rospy.get_param("~field", None)
        self.picker_ids = list(rospy.get_param("~picker_ids", []))
        self.transporter_ids = list(rospy.get_param("~transporter_ids", []))
        self.picker_base = rospy.get_param("~picker_base", None)
        self.picker_midpoint = rospy.get_param("~picker_midpoint", None)
        self.picker_endpoint = rospy.get_param("~picker_endpoint", None)
        self.transporter_initial = rospy.get_param("~transporter_initial_position", None)
        self.east_stops = rospy.get_param("~east_stops", None)
        self.picker_to_lane = rospy.get_param("~picker_to_lane", None)
        self.control = rospy.get_param("~control", None)

        required = {
            "field": self.field,
            "picker_ids": self.picker_ids,
            "transporter_ids": self.transporter_ids,
            "picker_base": self.picker_base,
            "picker_midpoint": self.picker_midpoint,
            "picker_endpoint": self.picker_endpoint,
            "transporter_initial_position": self.transporter_initial,
            "east_stops": self.east_stops,
            "picker_to_lane": self.picker_to_lane,
            "control": self.control,
        }
        missing = [k for k, v in required.items() if not v]
        if missing:
            raise RuntimeError("zones.yaml 缺少必需字段: {}".format(missing))

        self.pickers = ["robot{}".format(i) for i in self.picker_ids]
        self.transporters = ["robot{}".format(i) for i in self.transporter_ids]
        self.active = self.pickers + self.transporters

        self.safety_distance = float(self.control.get("safety_distance", 0.3))
        self.service_unload_sec = float(self.control.get("service_unload_sec",
                                                         self.UNLOAD_DURATION))

        # SimpleController 控制参数（从 launch arg 透传）
        self.ctrl_kwargs = dict(
            kp_linear=float(rospy.get_param("~Kp_linear", 0.5)),
            kp_angular=float(rospy.get_param("~Kp_angular", 1.0)),
            max_linear=float(rospy.get_param("~max_linear", 0.25)),
            max_angular=float(rospy.get_param("~max_angular", 0.8)),
            dist_tolerance=float(rospy.get_param("~dist_tolerance", 0.2)),
            yaw_tolerance=float(rospy.get_param("~yaw_tolerance", 0.3)),
            differential_mode=self.differential_mode,
        )

        # S2 Hungarian 配对结果（S4 直接复用，不重算）
        self.s2_pairs = []            # list[(transporter_id, picker_id)]
        self.served_this_round = set()

        # wait_rounds 相关字段保留但不再被调用（单轮流程下无意义）
        self.wait_rounds = {p: 0 for p in self.pickers}
        self.fairness_alpha = 2.5

        rospy.loginfo("[orchestrator] pickers=%s transporters=%s differential_mode=%s",
                      self.pickers, self.transporters, self.differential_mode)

        # 独立 pose 缓存（与 SimpleController 的订阅并行）
        self.pose_cache = {r: None for r in self.active}
        self._subs = []
        for rid in self.active:
            topic = "/mocap_node/Robot_{}/pose".format(_robot_id_to_int(rid))
            sub = rospy.Subscriber(topic, PoseStamped,
                                   self._make_pose_cb(rid), queue_size=1)
            self._subs.append(sub)
            rospy.loginfo("[orchestrator] 订阅 %s ← %s", rid, topic)

        self.phase_times = []
        self.stage_report = []
        self._active_controllers = []
        rospy.on_shutdown(self._shutdown_stop_all)

    # ------------------------------------------------------------------
    #  Pose 订阅
    # ------------------------------------------------------------------
    def _make_pose_cb(self, rid):
        def cb(msg):
            x = msg.pose.position.x
            y = msg.pose.position.y
            if abs(x) > 5000.0 or abs(y) > 5000.0:
                return
            yaw = quat_to_yaw(msg.pose.orientation)
            self.pose_cache[rid] = (x, y, yaw)
        return cb

    def _require_all_poses(self, timeout=10.0):
        t0 = rospy.Time.now()
        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown():
            missing = [r for r, p in self.pose_cache.items() if p is None]
            if not missing:
                return
            if (rospy.Time.now() - t0).to_sec() > timeout:
                raise RuntimeError("等待 pose 超时，缺失: {}".format(missing))
            rospy.logwarn_throttle(2.0, "[orchestrator] 等待 pose: %s", missing)
            rate.sleep()

    # ------------------------------------------------------------------
    #  日志辅助
    # ------------------------------------------------------------------
    def _stage_banner(self, name):
        rospy.loginfo("=" * 60)
        rospy.loginfo(">>> STAGE %s START", name)
        rospy.loginfo("=" * 60)

    def _stage_end(self, name, elapsed, timed_out):
        if timed_out:
            rospy.logerr("[!!!] STAGE %s TIMEOUT: robots %s", name, timed_out)
            result = "TIMEOUT {}".format(timed_out)
        else:
            result = "OK"
        rospy.loginfo("<<< STAGE %s END  (result: %s, elapsed=%.2fs)",
                      name, result, elapsed)

    def _log_goal_dispatch(self, phase_name, robot_id, gx, gy):
        pose = self.pose_cache.get(robot_id)
        if pose is None:
            rospy.loginfo("[%s] Robot %s goal=(%.2f,%.2f) from=None",
                          phase_name, robot_id, gx, gy)
            return
        cx, cy = pose[0], pose[1]
        d = math.hypot(gx - cx, gy - cy)
        rospy.loginfo("[%s] Robot %s goal=(%.2f,%.2f) from=(%.2f,%.2f) dist=%.2fm",
                      phase_name, robot_id, gx, gy, cx, cy, d)

    def _log_s2_hungarian(self, phase_name, wait_before, C, pairs,
                          wait_after, unserved):
        """按 prompt C 节格式打印 Hungarian 输入/输出。"""
        rospy.loginfo("[%s] Hungarian input:", phase_name)
        rospy.loginfo("  wait_rounds: %s",
                      {_robot_id_to_int(k): v for k, v in wait_before.items()})
        header = "           " + "".join(
            ["P{:<8d}".format(pid) for pid in self.picker_ids])
        rospy.loginfo("  cost_matrix (rows=transporters, cols=pickers):")
        rospy.loginfo("  %s", header)
        for i, tid in enumerate(self.transporter_ids):
            row_vals = "".join(["{:<9.2f}".format(C[i][j])
                                for j in range(len(self.picker_ids))])
            rospy.loginfo("    T{}   {}".format(tid, row_vals))
        assign_numeric = [(self.transporter_ids[ti], self.picker_ids[pi])
                          for ti, pi in pairs]
        rospy.loginfo("  → assignment: %s", assign_numeric)
        rospy.loginfo("  → wait_rounds after: %s",
                      {_robot_id_to_int(k): v for k, v in wait_after.items()})
        unserved_ids = [_robot_id_to_int(p) for p in unserved]
        rospy.loginfo("  → not served this round: %s", unserved_ids)

    def _print_run_summary(self, total):
        rospy.loginfo("=" * 60)
        rospy.loginfo("             RUN SUMMARY")
        rospy.loginfo("=" * 60)
        for item in self.stage_report:
            mark = "OK" if not item["timed_out"] else \
                   "TIMEOUT {}".format(item["timed_out"])
            rospy.loginfo("  %-30s %s", item["stage"], mark)
        rospy.loginfo("-" * 60)
        for name, t in self.phase_times:
            rospy.loginfo("  %-30s %.2fs", name, t)
        rospy.loginfo("  %-30s %.2fs", "TOTAL", total)
        rospy.loginfo("=" * 60)

    # ------------------------------------------------------------------
    #  阶段公共逻辑
    # ------------------------------------------------------------------
    def run_phase(self, phase_name, robot_ids, goals_list, task_labels,
                  assign=True, timeout_sec=None):
        """
        用于 S0/S1/S3/S5 这类并行单 waypoint 阶段。
        返回 goal_by_robot: {rid: (x, y, yaw)}
        """
        t0 = rospy.Time.now()

        positions = []
        for rid in robot_ids:
            p = self.pose_cache[rid]
            if p is None:
                raise RuntimeError("{} 没有 pose".format(rid))
            positions.append((p[0], p[1]))
        goal_xy = [(g[0], g[1]) for g in goals_list]

        if assign:
            C = build_cost_matrix(positions, goal_xy)
            pairs = assign_tasks(C)
            print_assignment_table(pairs, C,
                                   robot_ids=robot_ids,
                                   task_ids=task_labels,
                                   title="{} 分配".format(phase_name))
            goal_by_robot = {robot_ids[r]: goals_list[t] for r, t in pairs}
        else:
            if len(robot_ids) != len(goals_list):
                raise RuntimeError("assign=False 需要 robot_ids 与 goals_list 等长")
            goal_by_robot = dict(zip(robot_ids, goals_list))

        for rid in robot_ids:
            g = goal_by_robot[rid]
            self._log_goal_dispatch(phase_name, rid, g[0], g[1])

        controllers = {}
        for rid in robot_ids:
            gx, gy, gyaw = goal_by_robot[rid]
            controllers[rid] = SimpleController(
                mocap_topic="/mocap_node/Robot_{}/pose".format(_robot_id_to_int(rid)),
                cmd_vel_topic="/{}/cmd_vel".format(rid),
                goal_x=gx, goal_y=gy, goal_yaw=gyaw,
                robot_name=rid,
                **self.ctrl_kwargs,
            )
        self._active_controllers = list(controllers.values())

        if timeout_sec is None:
            timeout_sec = self.TIMEOUT_LANE
        try:
            _, timed_out = self.wait_until_arrived(
                list(controllers.values()),
                timeout_sec=timeout_sec,
                stage_name=phase_name,
            )
            self.stage_report.append({"stage": phase_name, "timed_out": timed_out})
        finally:
            for c in controllers.values():
                c.stop()
            self._active_controllers = []

        elapsed = (rospy.Time.now() - t0).to_sec()
        self.phase_times.append((phase_name, elapsed))
        self._stage_end(phase_name, elapsed, timed_out)
        return goal_by_robot

    def _check_safety(self, controllers):
        """同组两两距离 < safety_distance 时 warn-only。"""
        from itertools import combinations
        items = list(controllers.items())
        for (r1, c1), (r2, c2) in combinations(items, 2):
            if c1.current_pose is None or c2.current_pose is None:
                continue
            d = math.hypot(c1.current_pose[0] - c2.current_pose[0],
                           c1.current_pose[1] - c2.current_pose[1])
            if d < self.safety_distance:
                rospy.logwarn_throttle(
                    1.0,
                    "[safety] %s ↔ %s 距离 %.2fm < %.2fm（warn-only，不暂停车）",
                    r1, r2, d, self.safety_distance)

    def _print_status(self, phase_name, controllers):
        for rid, c in controllers.items():
            if c.current_pose is None:
                rospy.loginfo("[%s/%s] pose=None state=%s",
                              phase_name, rid, c.state)
                continue
            cx, cy, cyaw = c.current_pose
            vx, vy, wz = c.last_cmd
            rospy.loginfo("[%s/%s] state=%-8s pose=(%+.2f, %+.2f, %+.1f°) "
                          "dist=%.2f cmd=(%+.2f, %+.2f, %+.2f)",
                          phase_name, rid, c.state,
                          cx, cy, math.degrees(cyaw),
                          c.distance_to_goal, vx, vy, wz)

    # ------------------------------------------------------------------
    #  等待与超时
    # ------------------------------------------------------------------
    def wait_until_arrived(self, controllers, timeout_sec, stage_name=""):
        """
        等待 controllers 列表里所有实例 arrived=True，或超时。
        超时时对未到达的车调 c.stop() + WARN，返回 (False, [robot_name,...])。
        安全检查与 2s 状态打印折进循环。
        """
        start = rospy.Time.now()
        rate = rospy.Rate(10)
        ctrl_map = {c.robot_name: c for c in controllers}
        last_print = rospy.Time.now()
        while not rospy.is_shutdown():
            pending = [c for c in controllers if not c.arrived]
            if not pending:
                rospy.loginfo("[%s] 全员到达", stage_name)
                return True, []
            elapsed = (rospy.Time.now() - start).to_sec()
            if elapsed > timeout_sec:
                timed_out_names = []
                for c in pending:
                    rospy.logwarn("[%s] %s 等待 %.1fs 仍未到达，强制停车",
                                  stage_name, c.robot_name, timeout_sec)
                    c.stop()
                    timed_out_names.append(c.robot_name)
                return False, timed_out_names
            self._check_safety(ctrl_map)
            if (rospy.Time.now() - last_print).to_sec() >= 2.0:
                self._print_status(stage_name, ctrl_map)
                last_print = rospy.Time.now()
            rate.sleep()
        return False, [c.robot_name for c in controllers if not c.arrived]

    # ------------------------------------------------------------------
    #  Transporter 服务（S2/S4）辅助
    # ------------------------------------------------------------------
    def _execute_waypoint(self, robot_id, goal, phase_name, timeout_sec):
        """单台车单 waypoint：new 一个 SimpleController 等到达或超时。
        返回 (ok: bool, timed_out: list[robot_name])。
        """
        gx, gy, gyaw = goal
        self._log_goal_dispatch(phase_name, robot_id, gx, gy)
        ctrl = SimpleController(
            mocap_topic="/mocap_node/Robot_{}/pose".format(_robot_id_to_int(robot_id)),
            cmd_vel_topic="/{}/cmd_vel".format(robot_id),
            goal_x=gx, goal_y=gy, goal_yaw=gyaw,
            robot_name=robot_id,
            **self.ctrl_kwargs,
        )
        try:
            ok, timed_out = self.wait_until_arrived([ctrl], timeout_sec, phase_name)
        finally:
            ctrl.stop()
        return ok, timed_out

    def _serve_picker(self, transporter_id, picker_id, picker_pos,
                      phase_name, timeout_sec, results):
        """4-waypoint 服务序列，在独立线程跑。
           1. east_stops[lane]  2. stop_point  3. stop+sleep  4. east_stops[lane]
        """
        lane = self.picker_to_lane[picker_id]
        east_xy = tuple(self.east_stops[lane])
        stop_pt = (picker_pos[0] + self.safety_distance, picker_pos[1])
        service_log = []

        ok, to = self._execute_waypoint(
            transporter_id, (east_xy[0], east_xy[1], 0.0),
            "{}:WP1_east".format(phase_name), timeout_sec)
        service_log.append(("WP1", ok, to))

        ok, to = self._execute_waypoint(
            transporter_id, (stop_pt[0], stop_pt[1], 0.0),
            "{}:WP2_stop".format(phase_name), timeout_sec)
        service_log.append(("WP2", ok, to))

        rospy.loginfo("[%s] Robot %s unloading at (%.2f,%.2f) for %.1fs",
                      phase_name, transporter_id, stop_pt[0], stop_pt[1],
                      self.service_unload_sec)
        rospy.sleep(self.service_unload_sec)

        ok, to = self._execute_waypoint(
            transporter_id, (east_xy[0], east_xy[1], 0.0),
            "{}:WP4_return".format(phase_name), timeout_sec)
        service_log.append(("WP4", ok, to))

        results[transporter_id] = service_log

    def _run_transporter_services(self, pairs, picker_pos_by_id,
                                  phase_name, timeout_sec):
        """S2/S4 共用：每个 (t, p) 开一个线程跑 _serve_picker，主线程 join。"""
        t_start = rospy.Time.now()
        results = {}
        threads = []
        for t_id, p_id in pairs:
            picker_pos = picker_pos_by_id[p_id]
            th = threading.Thread(
                target=self._serve_picker,
                args=(t_id, p_id, picker_pos, phase_name, timeout_sec, results),
                name="serve-{}-{}".format(t_id, p_id),
            )
            threads.append(th)
            th.start()
        for th in threads:
            th.join()

        phase_timed_out = []
        for t_id, log in results.items():
            for wp, ok, to_list in log:
                if not ok:
                    if t_id not in phase_timed_out:
                        phase_timed_out.append(t_id)
                    rospy.logerr("[!!!] %s %s waypoint %s TIMEOUT",
                                 phase_name, t_id, wp)
        self.stage_report.append({"stage": phase_name,
                                  "timed_out": phase_timed_out})
        elapsed = (rospy.Time.now() - t_start).to_sec()
        self.phase_times.append((phase_name, elapsed))
        self._stage_end(phase_name, elapsed, phase_timed_out)

    # ------------------------------------------------------------------
    #  wait_rounds 更新（保留不调用，留给未来多轮流程）
    # ------------------------------------------------------------------
    def update_wait_rounds(self, assigned_picker_ids):
        for pid in self.wait_rounds:
            self.wait_rounds[pid] = 0 if pid in assigned_picker_ids \
                                       else self.wait_rounds[pid] + 1
        rospy.loginfo("[orchestrator] wait_rounds 更新后: %s", self.wait_rounds)

    # ------------------------------------------------------------------
    #  各阶段入口
    # ------------------------------------------------------------------
    def phase_gather(self):
        """S0: picker→picker_base, transporter→transporter_initial_position。"""
        goals = []
        labels = []
        robot_ids = list(self.active)
        for p in self.pickers:
            xy = self.picker_base[p]
            goals.append((xy[0], xy[1], 0.0))
            labels.append("base_" + p)
        for t in self.transporters:
            xy = self.transporter_initial[t]
            goals.append((xy[0], xy[1], 0.0))
            labels.append("init_" + t)
        self.run_phase("S0-gather", robot_ids, goals, labels,
                       assign=False, timeout_sec=self.TIMEOUT_LANE)

    def phase_pickers_to_midpoint(self):
        """S1: 3 picker 并行 → picker_midpoint。"""
        goals = [(self.picker_midpoint[p][0], self.picker_midpoint[p][1], 0.0)
                 for p in self.pickers]
        labels = ["mid_" + p for p in self.pickers]
        self.run_phase("S1-pickers_to_midpoint", self.pickers, goals, labels,
                       assign=False, timeout_sec=self.TIMEOUT_LANE)

    def phase_transport_midpoint(self):
        """S2: Hungarian 分配 2 transporter 服务 3 picker 中的 2 个，4-waypoint 服务。"""
        phase_name = "S2-transport_midpoint"

        transporter_xy = []
        for t in self.transporters:
            p = self.pose_cache[t]
            if p is None:
                raise RuntimeError("transporter {} 没有 pose".format(t))
            transporter_xy.append((p[0], p[1]))
        picker_mid_xy = [(self.picker_midpoint[p][0], self.picker_midpoint[p][1])
                         for p in self.pickers]

        wait_before = dict(self.wait_rounds)
        C = build_cost_matrix(transporter_xy, picker_mid_xy)  # 不传 wait_rounds/alpha
        pairs = assign_tasks(C)
        print_assignment_table(pairs, C,
                               robot_ids=self.transporters,
                               task_ids=self.pickers,
                               title="{} 分配".format(phase_name))

        self.s2_pairs = [(self.transporters[ti], self.pickers[pi])
                         for ti, pi in pairs]
        self.served_this_round = {p for (_, p) in self.s2_pairs}
        unserved = [p for p in self.pickers if p not in self.served_this_round]

        # prompt C 节格式（wait_rounds 未更新，after==before）
        wait_after = dict(self.wait_rounds)
        self._log_s2_hungarian(phase_name, wait_before, C, pairs,
                               wait_after, unserved)

        self._run_transporter_services(
            pairs=self.s2_pairs,
            picker_pos_by_id={p: tuple(self.picker_midpoint[p]) for p in self.pickers},
            phase_name=phase_name,
            timeout_sec=self.TIMEOUT_LANE,
        )

    def phase_pickers_to_endpoint(self):
        """S3: 3 picker 并行 → picker_endpoint。transporter 停在 east_stop 不动。"""
        goals = [(self.picker_endpoint[p][0], self.picker_endpoint[p][1], 0.0)
                 for p in self.pickers]
        labels = ["end_" + p for p in self.pickers]
        self.run_phase("S3-pickers_to_endpoint", self.pickers, goals, labels,
                       assign=False, timeout_sec=self.TIMEOUT_LANE)

    def phase_transport_endpoint(self):
        """S4: 复用 S2 配对，在 picker_endpoint 再服务一次（不跑 Hungarian）。"""
        phase_name = "S4-transport_endpoint"
        for p in self.pickers:
            if p not in self.served_this_round:
                ep = self.picker_endpoint[p]
                rospy.loginfo("[%s] Picker %s not served this round, "
                              "waiting at (%.2f, %.2f)",
                              phase_name, p, ep[0], ep[1])
        self._run_transporter_services(
            pairs=self.s2_pairs,
            picker_pos_by_id={p: tuple(self.picker_endpoint[p])
                              for p in self.pickers},
            phase_name=phase_name,
            timeout_sec=self.TIMEOUT_LANE,
        )

    def phase_return_to_base(self):
        """S5: picker → picker_base。transporter 停在当前 east_stop。"""
        goals = [(self.picker_base[p][0], self.picker_base[p][1], 0.0)
                 for p in self.pickers]
        labels = ["base_" + p for p in self.pickers]
        self.run_phase("S5-return_to_base", self.pickers, goals, labels,
                       assign=False, timeout_sec=self.TIMEOUT_RETURN)

    # ------------------------------------------------------------------
    #  主循环与收尾
    # ------------------------------------------------------------------
    def run(self):
        rospy.loginfo("[orchestrator] 等首帧 pose (3s)...")
        rospy.sleep(3.0)
        self._require_all_poses()
        rospy.loginfo("[orchestrator] 所有活跃机器人 pose 就位，开始编排")

        t_all = rospy.Time.now()

        if self.skip_gather:
            rospy.loginfo("[orchestrator] 跳过 S0-gather（skip_gather=true）")
        else:
            self._stage_banner("S0-gather");          self.phase_gather()
        self._stage_banner("S1-pickers_to_midpoint"); self.phase_pickers_to_midpoint()
        self._stage_banner("S2-transport_midpoint");  self.phase_transport_midpoint()
        self._stage_banner("S3-pickers_to_endpoint"); self.phase_pickers_to_endpoint()
        self._stage_banner("S4-transport_endpoint");  self.phase_transport_endpoint()
        self._stage_banner("S5-return_to_base");      self.phase_return_to_base()

        total = (rospy.Time.now() - t_all).to_sec()
        self._print_run_summary(total)

    def _shutdown_stop_all(self):
        for c in self._active_controllers:
            try:
                c.stop()
            except Exception:
                pass


def main():
    rospy.init_node("multi_robot_orchestrator")
    orch = Orchestrator()
    orch.run()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("[orchestrator] 致命错误: %s", e)
        raise
