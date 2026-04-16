#!/usr/bin/env python3
"""
orchestrator.py — 果园多机器人采摘-运输编排器

四阶段串行工作流：
  A1: 3 picker 匈牙利分配 row_north 任务（西→东过走廊）
  B1: 3 transporter 匈牙利分配 A1 放下的 3 个果筐
  A2: 3 picker 匈牙利分配 row_south 任务（东→西过走廊）
  B2: 3 transporter 匈牙利分配 A2 放下的 3 个果筐

本版本只实现串行模式（pipeline=false）。
"""

import threading
import time

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped
from tf.transformations import quaternion_from_euler

from hungarian_utils import hungarian_assign, print_assignment_table


def yaw_to_quaternion(yaw):
    q = quaternion_from_euler(0.0, 0.0, float(yaw))
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


class Orchestrator(object):

    # ------------------------------------------------------------------ init
    def __init__(self):
        rospy.init_node("orchard_orchestrator")
        self._load_config()

        # 共享状态
        self.lock = threading.Lock()
        self.robot_positions = {}   # ns -> (x, y)
        self.ready_boxes = {}       # task_id -> (x, y)
        self.stage_timings = {}     # stage -> seconds

        self._setup_pose_subscribers()
        self._setup_action_clients()

    def _load_config(self):
        self.depot = rospy.get_param("~depot")
        self.picking_tasks = rospy.get_param("~picking_tasks")
        self.picker_namespaces = [r["namespace"] for r in rospy.get_param("~picker_robots")]
        self.transporter_namespaces = [
            r["namespace"] for r in rospy.get_param("~transporter_robots")
        ]
        # 每台 transporter 独立的 depot slot，避免三车同一格 → 先到挡后到
        self.depot_slots = rospy.get_param("~depot_slots", {})
        for ns in self.transporter_namespaces:
            if ns not in self.depot_slots:
                rospy.logwarn(
                    "[orchestrator] %s 未配置 depot_slot，回落到共用 depot (会触发抢位)", ns
                )
                self.depot_slots[ns] = dict(self.depot)

        timing = rospy.get_param("~timing")
        self.pick_dwell = float(timing["pick_dwell_seconds"])
        self.load_dwell = float(timing["load_dwell_seconds"])
        self.unload_dwell = float(timing["unload_dwell_seconds"])
        self.goal_timeout = float(timing["goal_timeout_seconds"])

        self.pipeline = bool(rospy.get_param("~mode/pipeline", False))
        self.mb_wait_timeout = float(rospy.get_param("~move_base_wait_timeout", 30.0))

        rospy.loginfo("[orchestrator] 配置加载完成：%d picker + %d transporter，%d 个任务，pipeline=%s",
                      len(self.picker_namespaces), len(self.transporter_namespaces),
                      len(self.picking_tasks), self.pipeline)

    def _setup_pose_subscribers(self):
        for ns in self.picker_namespaces + self.transporter_namespaces:
            topic = "/{}/amcl_pose".format(ns)
            rospy.Subscriber(topic, PoseWithCovarianceStamped,
                             self._make_pose_cb(ns), queue_size=1)
            rospy.loginfo("[orchestrator] 订阅 %s", topic)

    def _make_pose_cb(self, ns):
        def cb(msg):
            with self.lock:
                self.robot_positions[ns] = (
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                )
        return cb

    def _setup_action_clients(self):
        self.action_clients = {}
        for ns in self.picker_namespaces + self.transporter_namespaces:
            action_ns = "{}/move_base".format(ns)
            client = actionlib.SimpleActionClient(action_ns, MoveBaseAction)
            rospy.loginfo("[orchestrator] 等待 %s ...", action_ns)
            if not client.wait_for_server(rospy.Duration(self.mb_wait_timeout)):
                raise RuntimeError(
                    "move_base not available for {} within {}s".format(ns, self.mb_wait_timeout)
                )
            self.action_clients[ns] = client
            rospy.loginfo("[orchestrator] %s 已连接", action_ns)

    # ---------------------------------------------------------------- basics
    def _make_goal(self, x, y, yaw):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(x)
        goal.target_pose.pose.position.y = float(y)
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation = yaw_to_quaternion(yaw)
        return goal

    def _send_and_wait(self, ns, x, y, yaw):
        client = self.action_clients[ns]
        goal = self._make_goal(x, y, yaw)
        rospy.loginfo("[%s] 发送目标 (x=%.2f, y=%.2f, yaw=%.2f)", ns, x, y, yaw)
        client.send_goal(goal)
        finished = client.wait_for_result(rospy.Duration(self.goal_timeout))
        if not finished:
            rospy.logwarn("[%s] 目标超时 %.0fs，取消", ns, self.goal_timeout)
            client.cancel_goal()
            return False
        state = client.get_state()
        if state == GoalStatus.SUCCEEDED:
            rospy.loginfo("[%s] 到达目标", ns)
            return True
        rospy.logwarn("[%s] 目标失败，state=%d", ns, state)
        return False

    def _get_robot_pos(self, ns, fallback):
        with self.lock:
            return self.robot_positions.get(ns, fallback)

    # ------------------------------------------------------- per-robot tasks
    def _run_picker_task(self, ns, task, results):
        task_id = task["id"]
        rospy.loginfo("[picker %s] 开始任务 %s", ns, task_id)

        e = task["entry"]
        if not self._send_and_wait(ns, e["x"], e["y"], e["yaw"]):
            rospy.logwarn("[picker %s] 进入 entry 失败 %s", ns, task_id)
            results[ns] = False
            return

        x = task["exit"]
        if not self._send_and_wait(ns, x["x"], x["y"], x["yaw"]):
            rospy.logwarn("[picker %s] 推进到 exit 失败 %s", ns, task_id)
            results[ns] = False
            return

        rospy.loginfo("[picker %s] 在行尾模拟采摘 %.1fs ...", ns, self.pick_dwell)
        rospy.sleep(self.pick_dwell)

        # 让位：退到 stash 点后再登记果筐，确保 transporter 到达 pickup 时不会
        # 把 picker 当成终点障碍 → move_base 规划失败（B 阶段全员 state=4 的根因）
        s = task.get("stash")
        if s is None:
            rospy.logwarn("[picker %s] 任务 %s 未配置 stash，跳过让位", ns, task_id)
        else:
            rospy.loginfo("[picker %s] 让位到 stash (%.2f, %.2f)", ns, s["x"], s["y"])
            if not self._send_and_wait(ns, s["x"], s["y"], s["yaw"]):
                rospy.logwarn("[picker %s] 任务 %s 让位失败，果筐仍登记但 pickup 可能阻塞",
                              ns, task_id)

        with self.lock:
            self.ready_boxes[task_id] = (float(x["x"]), float(x["y"]))
        rospy.loginfo("[picker %s] 完成 %s，果筐置于 (%.2f, %.2f)",
                      ns, task_id, x["x"], x["y"])
        results[ns] = True

    def _run_transporter_task(self, ns, task_id, pickup_xy, results):
        rospy.loginfo("[transporter %s] 接收 %s @ (%.2f, %.2f)",
                      ns, task_id, pickup_xy[0], pickup_xy[1])

        if not self._send_and_wait(ns, pickup_xy[0], pickup_xy[1], 0.0):
            rospy.logwarn("[transporter %s] 取货失败 %s", ns, task_id)
            results[ns] = False
            return
        rospy.loginfo("[transporter %s] 装载 %.1fs ...", ns, self.load_dwell)
        rospy.sleep(self.load_dwell)

        slot = self.depot_slots[ns]
        rospy.loginfo("[transporter %s] 返 depot_slot (%.2f, %.2f)", ns, slot["x"], slot["y"])
        if not self._send_and_wait(ns, slot["x"], slot["y"], slot["yaw"]):
            rospy.logwarn("[transporter %s] 回 depot 失败 %s", ns, task_id)
            results[ns] = False
            return
        rospy.loginfo("[transporter %s] 卸货 %.1fs ...", ns, self.unload_dwell)
        rospy.sleep(self.unload_dwell)

        rospy.loginfo("[transporter %s] 完成运输 %s", ns, task_id)
        results[ns] = True

    # --------------------------------------------------------- stage runners
    def execute_picking_round(self, round_num):
        stage = "A{}".format(round_num)
        rospy.loginfo("[orchestrator] ===== 阶段 %s 开始 =====", stage)
        t0 = time.time()

        tasks = [t for t in self.picking_tasks if t["round"] == round_num]
        task_ids = [t["id"] for t in tasks]
        task_targets = [(t["entry"]["x"], t["entry"]["y"]) for t in tasks]

        robot_pos = [self._get_robot_pos(ns, (0.5, 4.0)) for ns in self.picker_namespaces]
        assignment, cost_mat, _ = hungarian_assign(
            robot_pos, task_targets, self.picker_namespaces, task_ids
        )
        print_assignment_table(assignment, cost_mat, self.picker_namespaces, task_ids,
                               title="阶段 {} 采摘分配".format(stage))

        results = {}
        threads = []
        for ns, t_id in assignment.items():
            task = next(t for t in tasks if t["id"] == t_id)
            th = threading.Thread(
                target=self._run_picker_task,
                args=(ns, task, results),
                name="picker_{}_{}".format(ns, t_id),
            )
            th.daemon = True
            threads.append(th)
        for th in threads:
            th.start()
        for th in threads:
            th.join()

        self.stage_timings[stage] = time.time() - t0
        ok = all(results.values())
        rospy.loginfo("[orchestrator] ===== 阶段 %s 完成，耗时 %.1fs，成功=%s =====",
                      stage, self.stage_timings[stage], ok)

    def execute_transport_round(self, round_num):
        stage = "B{}".format(round_num)
        rospy.loginfo("[orchestrator] ===== 阶段 %s 开始 =====", stage)
        t0 = time.time()

        round_tasks = [t for t in self.picking_tasks if t["round"] == round_num]
        # 只处理实际已登记到 ready_boxes 的果筐，允许前一轮 A 部分失败
        with self.lock:
            available = [(t["id"], self.ready_boxes[t["id"]])
                         for t in round_tasks if t["id"] in self.ready_boxes]
        avail_ids = {tid for tid, _ in available}
        missing = [t["id"] for t in round_tasks if t["id"] not in avail_ids]
        if missing:
            rospy.logwarn("[orchestrator] %s 缺失果筐 %s，跳过这些任务",
                          stage, missing)
        if not available:
            rospy.logwarn("[orchestrator] %s 无可运果筐，阶段跳过", stage)
            self.stage_timings[stage] = time.time() - t0
            return

        task_ids = [tid for tid, _ in available]
        pickups = [xy for _, xy in available]

        # transporter 数量 ≥ 任务数时裁剪为方阵；当前场景 N==M 最常见
        n = len(task_ids)
        active_transporters = self.transporter_namespaces[:n]
        robot_pos = [
            self._get_robot_pos(ns, (self.depot_slots[ns]["x"], self.depot_slots[ns]["y"]))
            for ns in active_transporters
        ]
        assignment, cost_mat, _ = hungarian_assign(
            robot_pos, pickups, active_transporters, task_ids
        )
        print_assignment_table(assignment, cost_mat, active_transporters, task_ids,
                               title="阶段 {} 运输分配".format(stage))

        results = {}
        threads = []
        pickup_by_id = dict(zip(task_ids, pickups))
        for ns, t_id in assignment.items():
            pickup_xy = pickup_by_id[t_id]
            th = threading.Thread(
                target=self._run_transporter_task,
                args=(ns, t_id, pickup_xy, results),
                name="transporter_{}_{}".format(ns, t_id),
            )
            th.daemon = True
            threads.append(th)
        for th in threads:
            th.start()
        for th in threads:
            th.join()

        self.stage_timings[stage] = time.time() - t0
        ok = results and all(results.values())
        rospy.loginfo("[orchestrator] ===== 阶段 %s 完成，耗时 %.1fs，成功=%s =====",
                      stage, self.stage_timings[stage], ok)

    # --------------------------------------------------------------- driver
    def run(self):
        rospy.loginfo("[orchestrator] 等待 amcl_pose 初始化 3s ...")
        rospy.sleep(3.0)

        if self.pipeline:
            raise NotImplementedError(
                "pipeline=true 分支尚未实现；本次仅运行串行模式 pipeline=false"
            )

        t_total = time.time()
        self.execute_picking_round(1)
        self.execute_transport_round(1)
        self.execute_picking_round(2)
        self.execute_transport_round(2)
        total = time.time() - t_total
        self._print_summary(total)

    def _print_summary(self, total):
        lines = [
            "",
            "=" * 50,
            "[orchestrator] 全部任务完成",
            "=" * 50,
        ]
        for stage in ("A1", "B1", "A2", "B2"):
            dur = self.stage_timings.get(stage)
            if dur is not None:
                lines.append("  阶段 {}: {:.1f}s".format(stage, dur))
        lines.append("  总耗时: {:.1f}s".format(total))
        lines.append("  调度模式: {}".format("pipeline" if self.pipeline else "serial"))
        lines.append("=" * 50)
        for line in lines:
            rospy.loginfo("%s", line)


if __name__ == "__main__":
    try:
        Orchestrator().run()
    except rospy.ROSInterruptException:
        pass
