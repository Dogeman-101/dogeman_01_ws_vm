#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
simple_waypoint_runner.py — 单车多航点（动捕直控，不走 move_base）

从 rosparam 读 `test_sequence`，顺序创建 SimpleController 跑每个航点。
  - 每到一个点打印"航点 k/N 完成"
  - 全部完成打印汇总（总耗时 + 各航点耗时）

航点之间**新建**一个 SimpleController 实例（不改 goal），因为当前
SimpleController 没有 set_goal() 接口。老实例 stop() 发零速后由 Python
GC 回收；即便 Timer 延迟一拍，老实例 arrived=True 也只会发零速。

ROS 参数：
  ~mocap_topic      默认 /mocap_node/Robot_1/pose
  ~cmd_vel_topic    默认 /robot1/cmd_vel
  test_sequence     从 zones.yaml 加载（顶层参数）
"""

import math
import os
import sys

# 让脚本在"源码直跑"和"catkin 安装后 rosrun"两种模式下都能 import controller
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

from controller.simple_controller import SimpleController


def main():
    rospy.init_node("simple_waypoint_runner")

    mocap_topic = rospy.get_param("~mocap_topic", "/mocap_node/Robot_1/pose")
    cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/robot1/cmd_vel")
    # zones.yaml 由 launch 在 <node> 内用 <rosparam command="load"> 加载，
    # 顶层 key 落在节点私有命名空间，必须用 "~test_sequence" 读。
    waypoints = rospy.get_param("~test_sequence", None)
    if not waypoints:
        rospy.logerr("[W] 缺少 rosparam ~test_sequence，请确认 launch 在 <node> 内加载了 zones.yaml")
        return

    rospy.loginfo("[W] simple_waypoint_runner 启动。航点数 = %d", len(waypoints))
    for i, wp in enumerate(waypoints):
        rospy.loginfo("[W]   %d/%d: %s (%.3f, %.3f, yaw=%.3f rad / %.1f°)",
                      i + 1, len(waypoints),
                      wp.get("name", "?"),
                      wp["x"], wp["y"], wp["yaw"],
                      math.degrees(wp["yaw"]))

    total_start = rospy.Time.now()
    leg_times = []
    ctrl = None
    try:
        for i, wp in enumerate(waypoints):
            rospy.loginfo("[W] ========== 航点 %d/%d: %s ==========",
                          i + 1, len(waypoints), wp.get("name", "?"))

            # 老实例（如果有）先停车再替换
            if ctrl is not None:
                ctrl.stop()
                ctrl = None

            ctrl = SimpleController(
                mocap_topic=mocap_topic,
                cmd_vel_topic=cmd_vel_topic,
                goal_x=wp["x"], goal_y=wp["y"], goal_yaw=wp["yaw"],
                robot_name="W{}".format(i + 1),
            )
            rospy.on_shutdown(ctrl.stop)

            t_leg = rospy.Time.now()
            rate = rospy.Rate(1.0)
            tick = 0
            while not rospy.is_shutdown() and not ctrl.arrived:
                if ctrl.current_pose is None:
                    rospy.loginfo("[W] t=%ds 等待 mocap...", tick)
                else:
                    cx, cy, cyaw = ctrl.current_pose
                    rospy.loginfo("[W] t=%ds state=%s dist=%.3f "
                                  "pose=(%.3f, %.3f, %.1f°) cmd=(%+.3f, %+.3f, %+.3f)",
                                  tick, ctrl.state, ctrl.distance_to_goal,
                                  cx, cy, math.degrees(cyaw),
                                  ctrl.last_cmd[0], ctrl.last_cmd[1], ctrl.last_cmd[2])
                tick += 1
                rate.sleep()

            if rospy.is_shutdown():
                break

            leg_elapsed = (rospy.Time.now() - t_leg).to_sec()
            leg_times.append(leg_elapsed)
            rospy.loginfo("[W] 航点 %d/%d 完成，耗时 %.2fs",
                          i + 1, len(waypoints), leg_elapsed)
    finally:
        if ctrl is not None:
            ctrl.stop()

    total_elapsed = (rospy.Time.now() - total_start).to_sec()
    rospy.loginfo("[W] ==================== 汇总 ====================")
    for i, (wp, t) in enumerate(zip(waypoints, leg_times)):
        rospy.loginfo("[W]   %d/%d %-14s 耗时 %.2fs",
                      i + 1, len(waypoints), wp.get("name", "?"), t)
    rospy.loginfo("[W] 完成 %d/%d 航点，总耗时 %.2fs",
                  len(leg_times), len(waypoints), total_elapsed)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
