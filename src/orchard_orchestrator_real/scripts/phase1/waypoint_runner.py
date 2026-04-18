#!/usr/bin/env python3
"""
waypoint_runner.py — 场景 B：航点序列

用途：单点跑通后，给一串坐标让车按顺序走一圈，验证连续导航的可靠性。

改路线只需改下方 WAYPOINTS 列表，无需 catkin build。

前置：
  VM 侧   : roslaunch orchard_map vm_master.launch
  W4A 侧  : roslaunch w4a_bringup_nav.launch
"""

# =====================================================================
#  航点列表（改这里就换路线；元组：名字, x, y, yaw-弧度）
#  当前示例：绕原点 1m 方框一圈回 home
# =====================================================================
WAYPOINTS = [
    ("home",    0.0, 0.0, 0.0),
    ("point_A", 1.0, 0.0, 0.0),
    ("point_B", 1.0, 1.0, 1.5708),
    ("point_C", 0.0, 1.0, 3.1416),
    ("home_back", 0.0, 0.0, 0.0),
]

MAX_RETRY = 1      # 单航点失败后的重试次数
GOAL_TIMEOUT = 60.0
# =====================================================================

import math
import os
import sys

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

from utils.move_base_client import MoveBaseClient
from utils.tf_helper import TFHelper


def main():
    rospy.init_node("waypoint_runner")
    rospy.loginfo("[B] waypoint_runner 启动，共 %d 个航点", len(WAYPOINTS))

    tfh = TFHelper()
    if not tfh.wait_ready(timeout=10.0):
        rospy.logerr("[B] TF 未就绪，退出。")
        return

    mbc = MoveBaseClient()
    if not mbc.wait_server(timeout=10.0):
        rospy.logerr("[B] move_base 未响应，退出。")
        return

    rospy.on_shutdown(mbc.cancel)

    success_count = 0
    failed = []   # (name, state_name)

    total = len(WAYPOINTS)
    for idx, (name, x, y, yaw) in enumerate(WAYPOINTS, start=1):
        rospy.loginfo("[B] ========== 第 %d/%d 航点: %s  (x=%.3f, y=%.3f, yaw=%.1f°) ==========",
                      idx, total, name, x, y, math.degrees(yaw))
        tfh.print_pose(prefix="[B] 出发前")
        tfh.print_bearing_to(x, y, prefix="[B] 出发前")

        attempt = 0
        last_state_name = "UNKNOWN"
        while attempt <= MAX_RETRY:
            if attempt > 0:
                rospy.logwarn("[B]   重试 %d/%d", attempt, MAX_RETRY)
            ok, state, state_name = mbc.go_to(x, y, yaw, timeout=GOAL_TIMEOUT)
            last_state_name = state_name
            if ok:
                rospy.loginfo("[B] ✓ 到达 %s (state=%s)", name, state_name)
                success_count += 1
                tfh.print_pose(prefix="[B] 到达后")
                break
            rospy.logwarn("[B] ✗ %s 失败 state=%d (%s)", name, state, state_name)
            attempt += 1
        else:
            failed.append((name, last_state_name))
            rospy.logwarn("[B] 跳过 %s，继续下一个", name)

        if rospy.is_shutdown():
            break

    rospy.loginfo("[B] ==================== 汇总 ====================")
    rospy.loginfo("[B] 成功: %d / %d", success_count, total)
    rospy.loginfo("[B] 失败: %d", len(failed))
    for name, s in failed:
        rospy.loginfo("[B]   - %s : %s", name, s)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
