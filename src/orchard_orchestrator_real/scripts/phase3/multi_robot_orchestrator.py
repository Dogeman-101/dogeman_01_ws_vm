#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
multi_robot_orchestrator.py — 6 车采摘-运输编排（动捕直控版）

4 阶段串行：
  阶段 0  集合      : 所有车 Hungarian → base slot
  阶段 1  采摘出发  : picker   Hungarian → pick_targets
  阶段 2  运输出发  : transporter Hungarian → picker 停留点
  阶段 3  运输返回  : transporter → 各自 base slot（yaw=掉头）
  阶段 4  采摘返回  : picker      → 各自 base slot（yaw=掉头）

num_robots=3 时跳过 2/3（只跑 picker 流程）。

控制层：每阶段为活跃车新建 SimpleController；阶段结束 stop() 并弃用。
订阅层：自建 /mocap_node/Robot_N/pose 订阅做 Hungarian 的 pose 输入，
        与 SimpleController 对同一 topic 的订阅互不干扰。

安全距离：warn-only。SimpleController 没有 pause 接口，强行 stop() 会被
下一 tick 覆盖；参见计划文件的"与 spec 偏离"段。
"""

import math
import os
import sys
from itertools import combinations

# sys.path shim：让脚本在源码直跑和 install 后都能 import controller / algorithms
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


class Orchestrator(object):
    # 阶段超时常量（秒）——统一在这里调
    TIMEOUT_LANE    = 30.0   # S0/S1/S2：集合 / 走过道去采摘点 / 去服务 picker
    TIMEOUT_RETURN  = 40.0   # S3/S4：transporter / picker 返 base（路径较长）
    UNLOAD_DURATION = 5.0    # 卸货停留时长（预留，当前未调用）

    def __init__(self):
        self.num_robots = int(rospy.get_param("~num_robots", 6))
        self.skip_gather = bool(rospy.get_param("~skip_gather", False))
        self.differential_mode = bool(rospy.get_param("~differential_mode", True))

        # zones.yaml 在 launch 的 <node> 内用 <rosparam command="load"> 加载，
        # 顶层 key 落在节点私有命名空间，必须用 "~" 前缀读。
        self.base = rospy.get_param("~base", None)
        self.pick_targets = rospy.get_param("~pick_targets", None)
        self.roles = rospy.get_param("~roles", None)
        self.return_yaw = float(rospy.get_param("~return_yaw", 3.14159))
        self.safety_distance = float(rospy.get_param("~safety_distance", 0.5))

        if not self.base or not self.pick_targets or not self.roles:
            raise RuntimeError("缺少 zones.yaml 参数（base / pick_targets / roles）；"
                               "请确认 launch 里 <rosparam command=\"load\" file=...>")

        self.pickers = list(self.roles["picker"])
        self.transporters = list(self.roles["transporter"])

        self.active = self.pickers + self.transporters
        self.skip_transport = (len(self.transporters) == 0)

        self.wait_rounds = {pid: 0 for pid in self.pickers}
        self.fairness_alpha = 2.5

        rospy.loginfo("[orchestrator] num_robots=%d skip_gather=%s differential_mode=%s",
                      self.num_robots, self.skip_gather, self.differential_mode)
        rospy.loginfo("[orchestrator] 活跃机器人: %s", self.active)

        # 独立 pose 缓存（与 SimpleController 的订阅并行）
        self.pose_cache = {r: None for r in self.active}
        self._subs = []
        for rid in self.active:
            idx = int(rid.replace("robot", ""))
            topic = "/mocap_node/Robot_{}/pose".format(idx)
            sub = rospy.Subscriber(topic, PoseStamped,
                                   self._make_pose_cb(rid), queue_size=1)
            self._subs.append(sub)
            rospy.loginfo("[orchestrator] 订阅 %s ← %s", rid, topic)

        # 阶段 0 的分配结果（用于阶段 3/4 的返程闭环）
        self.base_assignment = {}
        # 阶段耗时记录
        self.phase_times = []
        # 阶段到达 / 超时报告：[{"stage": name, "timed_out": [robot_name,...]}]
        self.stage_report = []
        # 活跃 controllers（供 shutdown hook 用）
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
                return  # 动捕丢失异常值
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
    #  阶段公共逻辑
    # ------------------------------------------------------------------
    def run_phase(self, phase_name, robot_ids, goals_list, task_labels,
                  assign=True, timeout_sec=None):
        """
        robot_ids:   list[str]，本阶段参与的机器人
        goals_list:  list[(x, y, yaw)]，候选目标
        task_labels: list[str]，目标 id（打印用）
        assign=True: Hungarian 分配；False: robot[i] → goals_list[i] 一一对应
        返回 goal_by_robot: {rid: (x, y, yaw)}
        """
        rospy.loginfo("\n[orchestrator] ===== 阶段 %s 开始 =====", phase_name)
        t0 = rospy.Time.now()

        # 取当前 pose
        positions = []
        for rid in robot_ids:
            p = self.pose_cache[rid]
            if p is None:
                raise RuntimeError("{} 没有 pose".format(rid))
            positions.append((p[0], p[1]))
        goal_xy = [(g[0], g[1]) for g in goals_list]

        # 分配
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
            rospy.loginfo("[orchestrator] %s 直接对应（非 Hungarian）:", phase_name)
            for rid in robot_ids:
                g = goal_by_robot[rid]
                rospy.loginfo("[orchestrator]   %s → (%.3f, %.3f, yaw=%.3f)",
                              rid, g[0], g[1], g[2])

        # 实例化 controllers
        controllers = {}
        for rid in robot_ids:
            gx, gy, gyaw = goal_by_robot[rid]
            idx = int(rid.replace("robot", ""))
            controllers[rid] = SimpleController(
                mocap_topic="/mocap_node/Robot_{}/pose".format(idx),
                cmd_vel_topic="/{}/cmd_vel".format(rid),
                goal_x=gx, goal_y=gy, goal_yaw=gyaw,
                robot_name=rid,
                differential_mode=self.differential_mode,
            )
        self._active_controllers = list(controllers.values())

        # 等待 + 超时保护（_check_safety / 2s 状态打印折进 wait_until_arrived）
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
        rospy.loginfo("[orchestrator] ===== 阶段 %s 完成，耗时 %.2fs =====",
                      phase_name, elapsed)
        return goal_by_robot

    def _check_safety(self, controllers):
        """同组两两距离 < safety_distance 时 warn-only（见计划 'safety_distance' 段）。"""
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
        把原 run_phase 等待循环里的 _check_safety 与 2s _print_status 折进来。
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
        # rospy.is_shutdown()：视作"全部超时"
        return False, [c.robot_name for c in controllers if not c.arrived]

    # ------------------------------------------------------------------
    #  各阶段入口
    # ------------------------------------------------------------------
    def phase_gather(self):
        """阶段 0：所有活跃车 Hungarian 分到 base slots。"""
        slot_names = sorted(self.base.keys())[:len(self.active)]
        goals = [(self.base[s]["x"], self.base[s]["y"], self.base[s]["yaw"])
                 for s in slot_names]
        self.base_assignment = self.run_phase(
            "0-集合", self.active, goals, slot_names,
            assign=True, timeout_sec=self.TIMEOUT_LANE)

    def phase_pick_out(self):
        """阶段 1：3 picker Hungarian 分到 3 个 pick_targets。"""
        target_names = sorted(self.pick_targets.keys())
        goals = [(self.pick_targets[t]["x"],
                  self.pick_targets[t]["y"],
                  self.pick_targets[t]["yaw"]) for t in target_names]
        self.run_phase("1-采摘出发", self.pickers, goals, target_names,
                       assign=True, timeout_sec=self.TIMEOUT_LANE)

    def update_wait_rounds(self, assigned_picker_ids):
        for pid in self.wait_rounds:
            self.wait_rounds[pid] = 0 if pid in assigned_picker_ids \
                                       else self.wait_rounds[pid] + 1
        rospy.loginfo("[orchestrator] wait_rounds 更新后: %s", self.wait_rounds)

    def phase_transport_out(self):
        """阶段 2：transporter Hungarian 分到 picker 实际停留点（含 wait_rounds 公平惩罚）。"""
        pickup_points = []
        labels = []
        for p in self.pickers:
            pose = self.pose_cache[p]
            if pose is None:
                raise RuntimeError("picker {} 没有 pose".format(p))
            pickup_points.append((pose[0] - 0.4, pose[1], 0.0))  # 到达 picker 时朝东
            labels.append("pickup_{}".format(p))

        rospy.loginfo("[orchestrator] wait_rounds 分配前: %s", self.wait_rounds)
        transporter_xy = []
        for t in self.transporters:
            pose = self.pose_cache[t]
            if pose is None:
                raise RuntimeError("transporter {} 没有 pose".format(t))
            transporter_xy.append((pose[0], pose[1]))
        pickup_xy = [(pt[0], pt[1]) for pt in pickup_points]

        C = build_cost_matrix(transporter_xy, pickup_xy,
                              task_ids=self.pickers,
                              wait_rounds=self.wait_rounds,
                              alpha=self.fairness_alpha)
        pairs = assign_tasks(C)
        print_assignment_table(pairs, C,
                               robot_ids=self.transporters,
                               task_ids=labels,
                               title="2-运输出发 分配（含 wait_rounds）")
        assigned_pids = {self.pickers[t_idx] for _, t_idx in pairs}
        self.update_wait_rounds(assigned_pids)

        ordered_goals = [None] * len(self.transporters)
        ordered_labels = [None] * len(self.transporters)
        for r_idx, t_idx in pairs:
            ordered_goals[r_idx] = pickup_points[t_idx]
            ordered_labels[r_idx] = labels[t_idx]
        self.run_phase("2-运输出发", self.transporters,
                       ordered_goals, ordered_labels,
                       assign=False, timeout_sec=self.TIMEOUT_LANE)

    def phase_transport_back(self):
        """阶段 3：transporter 各回阶段 0 分配的 base slot（yaw=掉头）。"""
        goals = []
        for t in self.transporters:
            base_xyz = self.base_assignment.get(t)
            if base_xyz is None:
                # 跳过阶段 0 时 fallback：picker 占前几个 slot，transporter 从之后开始
                idx = self.transporters.index(t)
                slot_keys = sorted(self.base.keys())
                fallback_key = slot_keys[len(self.pickers) + idx]
                fb = self.base[fallback_key]
                base_xyz = (fb["x"], fb["y"], fb["yaw"])
            goals.append((base_xyz[0], base_xyz[1], self.return_yaw))
        self.run_phase("3-运输返回", self.transporters,
                       goals, ["base"] * len(self.transporters),
                       assign=False, timeout_sec=self.TIMEOUT_RETURN)

    def phase_pick_back(self):
        """阶段 4：picker 各回阶段 0 分配的 base slot（yaw=掉头）。"""
        goals = []
        for p in self.pickers:
            base_xyz = self.base_assignment.get(p)
            if base_xyz is None:
                # 如果跳过了阶段 0，fallback 到前三个 slot
                fallback = sorted(self.base.keys())[self.pickers.index(p)]
                fb = self.base[fallback]
                base_xyz = (fb["x"], fb["y"], fb["yaw"])
            goals.append((base_xyz[0], base_xyz[1], self.return_yaw))
        self.run_phase("4-采摘返回", self.pickers,
                       goals, ["base"] * len(self.pickers),
                       assign=False, timeout_sec=self.TIMEOUT_RETURN)

    # ------------------------------------------------------------------
    #  主循环与收尾
    # ------------------------------------------------------------------
    def run(self):
        rospy.loginfo("[orchestrator] 等首帧 pose（3s）...")
        rospy.sleep(3.0)
        self._require_all_poses()
        rospy.loginfo("[orchestrator] 所有活跃机器人 pose 就位，开始编排")

        t_all = rospy.Time.now()

        if self.skip_gather:
            rospy.loginfo("[orchestrator] 跳过阶段 0（skip_gather=true）")
        else:
            self.phase_gather()

        self.phase_pick_out()

        if not self.skip_transport:
            self.phase_transport_out()
            self.phase_transport_back()
        else:
            rospy.loginfo("[orchestrator] 跳过阶段 2/3（roles 无 transporter）")

        self.phase_pick_back()

        total = (rospy.Time.now() - t_all).to_sec()
        rospy.loginfo("\n[orchestrator] ==================== 汇总 ====================")
        for name, t in self.phase_times:
            rospy.loginfo("[orchestrator]   阶段 %-12s  %.2fs", name, t)
        rospy.loginfo("[orchestrator] 总耗时 %.2fs", total)

        rospy.loginfo("=" * 50)
        rospy.loginfo("=== Run Summary ===")
        for item in self.stage_report:
            mark = "OK" if not item["timed_out"] else "TIMEOUT {}".format(item["timed_out"])
            rospy.loginfo("  %s: %s", item["stage"], mark)
        rospy.loginfo("=" * 50)

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
