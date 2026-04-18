#!/usr/bin/env python3
"""
single_goal_debug.py — 场景 A：单点诊断

用途：动捕标定修好后第一件事。发一个固定点，逐秒打印诊断信息，
排查"给正前方一个点，车反而往后跑"这类问题。

改目标只需改下方三个常量，无需 catkin build（catkin_install_python
在 devel 下是符号链接）。

前置：
  VM 侧   : roslaunch orchard_map vm_master.launch
  W4A 侧  : roslaunch w4a_bringup_nav.launch
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

# 让脚本在"源码直跑"和"catkin 安装后 rosrun"两种模式下都能 import utils
# - 源码直跑：os.path.dirname(__file__) 是 .../scripts/phase1
# - install:  rospkg 定位到源码树根，再 +scripts
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
from geometry_msgs.msg import Twist

from utils.move_base_client import MoveBaseClient, STATE_NAMES, TERMINAL_STATES
from utils.tf_helper import TFHelper


class CmdVelMonitor(object):
    """订阅 /cmd_vel 缓存最新值；默认全 0。麦轮车要看 linear.y。"""

    def __init__(self, topic="/cmd_vel"):
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.sub = rospy.Subscriber(topic, Twist, self._cb, queue_size=1)

    def _cb(self, msg):
        self.linear_x = msg.linear.x
        self.linear_y = msg.linear.y
        self.angular_z = msg.angular.z


def main():
    rospy.init_node("single_goal_debug")
    rospy.loginfo("[A] single_goal_debug 启动。目标 (%.3f, %.3f, yaw=%.3f rad / %.1f°)",
                  GOAL_X, GOAL_Y, GOAL_YAW, math.degrees(GOAL_YAW))

    # 1. TF 就绪
    tfh = TFHelper()
    if not tfh.wait_ready(timeout=10.0):
        rospy.logerr("[A] TF 未就绪，退出。检查 VM 侧 mocap_to_tf / W4A 侧 EKF。")
        return

    # 2. move_base 就绪
    mbc = MoveBaseClient()
    if not mbc.wait_server(timeout=10.0):
        rospy.logerr("[A] move_base 未响应，退出。检查 W4A 上 navigation 是否在跑。")
        return

    # 3. 启动诊断（一次性）
    rospy.loginfo("[A] ---------- 启动诊断 ----------")
    tfh.print_pose(prefix="[A] 当前")
    tfh.print_bearing_to(GOAL_X, GOAL_Y, prefix="[A] 起始")

    # 4. /cmd_vel 监听
    cv = CmdVelMonitor("/cmd_vel")

    # 5. Ctrl-C 保护：退出时取消 goal
    rospy.on_shutdown(mbc.cancel)

    # 6. 非阻塞发 goal
    mbc.send_goal(GOAL_X, GOAL_Y, GOAL_YAW)

    # 7. 1Hz 轮询打印，直到终态
    rate = rospy.Rate(1.0)
    tick = 0
    while not rospy.is_shutdown():
        state = mbc.get_state()
        name = STATE_NAMES.get(state, "UNKNOWN")
        rospy.loginfo("[A] ---------- t=%ds  state=%d (%s) ----------", tick, state, name)
        tfh.print_pose(prefix="[A]")
        tfh.print_bearing_to(GOAL_X, GOAL_Y, prefix="[A]")
        rospy.loginfo("[A] /cmd_vel  linear.x=%+.3f  linear.y=%+.3f  angular.z=%+.3f",
                      cv.linear_x, cv.linear_y, cv.angular_z)
        if state in TERMINAL_STATES:
            break
        tick += 1
        rate.sleep()

    # 8. 收尾
    final_state = mbc.get_state()
    final_name = STATE_NAMES.get(final_state, "UNKNOWN")
    rospy.loginfo("[A] ==================== 结束 ====================")
    tfh.print_pose(prefix="[A] 最终")
    tfh.print_bearing_to(GOAL_X, GOAL_Y, prefix="[A] 最终")
    if final_state == 3:
        rospy.loginfo("[A] 结果：SUCCEEDED ✓")
    else:
        rospy.logwarn("[A] 结果：%s（state=%d）", final_name, final_state)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
