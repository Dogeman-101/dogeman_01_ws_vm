#!/usr/bin/env python3
"""
task_assigner.py — 果园多机器人任务分配节点

算法：同时向所有机器人发送目标（并发），等待全部完成后退出。
接口：从 ROS 参数 ~robots 加载机器人列表，每台机器人连接自己的
      {namespace}/move_base action server。
"""

import threading
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler


def yaw_to_quaternion(yaw):
    q = quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def dispatch_robot(ns, goal_cfg, timeout, results, idx):
    """在独立线程中连接一台机器人的 move_base 并等待目标完成。"""
    action_ns = '{}/move_base'.format(ns)
    client = actionlib.SimpleActionClient(action_ns, MoveBaseAction)

    rospy.loginfo("[%s] Connecting to %s ...", ns, action_ns)
    if not client.wait_for_server(rospy.Duration(30.0)):
        rospy.logerr("[%s] move_base not available after 30s.", ns)
        results[idx] = 'CONNECT_TIMEOUT'
        return

    x   = float(goal_cfg['x'])
    y   = float(goal_cfg['y'])
    yaw = float(goal_cfg.get('yaw', 0.0))

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp    = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation = yaw_to_quaternion(yaw)

    rospy.loginfo("[%s] Sending goal  (x=%.2f, y=%.2f, yaw=%.2f)", ns, x, y, yaw)
    client.send_goal(goal)

    finished = client.wait_for_result(rospy.Duration(timeout))
    if not finished:
        rospy.logwarn("[%s] Goal timed out (%.0fs). Cancelling.", ns, timeout)
        client.cancel_goal()
        results[idx] = 'GOAL_TIMEOUT'
        return

    state = client.get_state()
    if state == GoalStatus.SUCCEEDED:
        rospy.loginfo("[%s] SUCCEEDED.", ns)
        results[idx] = 'SUCCEEDED'
    else:
        rospy.logwarn("[%s] ended with state %d.", ns, state)
        results[idx] = 'FAILED:{}'.format(state)


def main():
    rospy.init_node('task_assigner')

    robots  = rospy.get_param('~robots', [])
    timeout = rospy.get_param('~goal_timeout', 180.0)

    if not robots:
        rospy.logwarn("[task_assigner] No robots configured on param ~robots. Exiting.")
        return

    rospy.loginfo("[task_assigner] Dispatching goals to %d robots (timeout=%.0fs each).",
                  len(robots), timeout)

    results = [None] * len(robots)
    threads = []

    for i, robot in enumerate(robots):
        ns       = robot['namespace']
        goal_cfg = robot['goal']
        t = threading.Thread(
            target=dispatch_robot,
            args=(ns, goal_cfg, timeout, results, i),
            name='dispatch_{}'.format(ns)
        )
        t.daemon = True
        threads.append(t)

    # Launch all robots simultaneously
    for t in threads:
        t.start()

    # Wait for all to finish
    for t in threads:
        t.join()

    rospy.loginfo("[task_assigner] ===== All robots done =====")
    for i, robot in enumerate(robots):
        rospy.loginfo("[task_assigner]   %-10s  %s", robot['namespace'], results[i])


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
