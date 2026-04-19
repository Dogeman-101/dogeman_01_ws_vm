#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
single_goal_simple.py — 单车单点（动捕直控，不走 move_base）

改目标只需改下方三个常量。ROS 参数：
  ~mocap_topic    默认 /mocap_node/Robot_1/pose
  ~cmd_vel_topic  默认 /robot1/cmd_vel

前置：
  VM 侧 : roslaunch orchard_map vm_master.launch
  W4A   : w4a_simple.launch（订 /robot1/cmd_vel）
"""

# =====================================================================
#  目标坐标（改这三行就换点，不要改别处）
# =====================================================================
GOAL_X = 1.0
GOAL_Y = 0.0
GOAL_YAW = 0.0   # 弧度
# =====================================================================

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
    rospy.init_node("single_goal_simple")

    mocap_topic = rospy.get_param("~mocap_topic", "/mocap_node/Robot_1/pose")
    cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/robot1/cmd_vel")

    rospy.loginfo("[S] single_goal_simple 启动。目标 (%.3f, %.3f, yaw=%.3f rad / %.1f°)",
                  GOAL_X, GOAL_Y, GOAL_YAW, math.degrees(GOAL_YAW))

    ctrl = SimpleController(
        mocap_topic=mocap_topic,
        cmd_vel_topic=cmd_vel_topic,
        goal_x=GOAL_X, goal_y=GOAL_Y, goal_yaw=GOAL_YAW,
        robot_name="S",
    )
    rospy.on_shutdown(ctrl.stop)

    t_start = rospy.Time.now()
    rate = rospy.Rate(1.0)
    tick = 0
    while not rospy.is_shutdown():
        rospy.loginfo("[S] ---------- t=%ds  state=%s ----------", tick, ctrl.state)
        if ctrl.current_pose is None:
            rospy.loginfo("[S] 当前位姿：等待 mocap ...")
        else:
            cx, cy, cyaw = ctrl.current_pose
            dx = GOAL_X - cx
            dy = GOAL_Y - cy
            bearing = math.atan2(dy, dx)
            rospy.loginfo("[S] 当前   x=%+.3f  y=%+.3f  yaw=%+.3f rad (%+.1f°)",
                          cx, cy, cyaw, math.degrees(cyaw))
            rospy.loginfo("[S]        距离=%+.3f m  方位=%+.3f rad (%+.1f°)",
                          ctrl.distance_to_goal, bearing, math.degrees(bearing))
        vx, vy, wz = ctrl.last_cmd
        rospy.loginfo("[S] /cmd_vel  linear.x=%+.3f  linear.y=%+.3f  angular.z=%+.3f",
                      vx, vy, wz)
        if ctrl.arrived:
            break
        tick += 1
        rate.sleep()

    elapsed = (rospy.Time.now() - t_start).to_sec()
    rospy.loginfo("[S] ==================== 结束 ====================")
    if ctrl.current_pose is not None:
        cx, cy, cyaw = ctrl.current_pose
        rospy.loginfo("[S] 最终   x=%+.3f  y=%+.3f  yaw=%+.3f rad (%+.1f°)",
                      cx, cy, cyaw, math.degrees(cyaw))
    rospy.loginfo("[S] 耗时 %.2f s", elapsed)
    if ctrl.arrived:
        rospy.loginfo("[S] 结果：SUCCEEDED ✓")
    else:
        rospy.logwarn("[S] 结果：中断（state=%s）", ctrl.state)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
