#!/usr/bin/env python3
"""
task_assigner_hungarian.py — 果园多机器人任务分配节点（匈牙利算法版）

算法流程：
  1. 从参数服务器读取机器人起始位置列表（~robots）和目标点列表（~tasks）
  2. 计算欧氏距离代价矩阵 C，其中 C[i][j] 表示第 i 台机器人走到第 j 个目标的直线距离
  3. 调用 scipy 的匈牙利算法求解最优分配，使总距离最小
  4. 打印分配结果供调试确认
  5. 并发向所有机器人发送 move_base 目标，等待全部完成后退出

依赖：scipy、numpy（通过 pip 安装，非 ROS 包）
  pip install scipy numpy
"""

import math
import threading

import numpy as np
from scipy.optimize import linear_sum_assignment

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler


# ---------------------------------------------------------------------------
# 工具函数（与 task_assigner.py 保持一致，独立复制，不 import 原文件）
# ---------------------------------------------------------------------------

def yaw_to_quaternion(yaw):
    """将偏航角（弧度）转换为四元数，供 MoveBaseGoal 使用。"""
    q = quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def dispatch_robot(ns, goal_cfg, timeout, results, idx):
    """
    在独立线程中连接一台机器人的 move_base 并等待目标完成。

    参数：
      ns        - 机器人命名空间，如 "robot1"
      goal_cfg  - 目标字典，含 'x'、'y'、'yaw' 三个键
      timeout   - 等待结果的最长秒数
      results   - 共享结果列表（线程安全写不同下标）
      idx       - 本线程写入 results[idx]
    """
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


# ---------------------------------------------------------------------------
# 匈牙利算法：构造代价矩阵并求最优分配
# ---------------------------------------------------------------------------

def solve_assignment(robots, tasks):
    """
    用匈牙利算法计算机器人到任务的最优（总代价最小）一对一分配。

    代价矩阵 C 的形状为 (n_robots, n_tasks)，n_robots == n_tasks。
    C[i][j] = 第 i 台机器人的出发点到第 j 个任务目标点的欧氏距离。
    欧氏距离是"第一层"代价，后续可升级为调用 move_base make_plan 服务
    获取真实路径长度，但不影响这里的算法接口。

    返回：
      col_ind  - 长度为 n 的数组，col_ind[i] 是分配给第 i 台机器人的任务索引
      C        - 代价矩阵本身（供打印用）
    """
    n = len(robots)

    # 构造 n×n 代价矩阵
    # C[i][j] = robot i 从 start(xi, yi) 到 task j 的目标点 (xj, yj) 的直线距离
    C = np.zeros((n, n), dtype=float)
    for i, robot in enumerate(robots):
        sx = float(robot['start']['x'])
        sy = float(robot['start']['y'])
        for j, task in enumerate(tasks):
            tx = float(task['x'])
            ty = float(task['y'])
            C[i][j] = math.sqrt((sx - tx) ** 2 + (sy - ty) ** 2)

    # linear_sum_assignment 实现 Jonker-Volgenant 算法（匈牙利算法的高效变体）。
    # 它接受代价矩阵 C，返回两个等长数组：
    #   row_ind[k], col_ind[k] 表示第 k 对分配
    # 由于 C 是方阵且行列一一对应，row_ind 恰好是 [0, 1, 2, ..., n-1]，
    # 因此只需关注 col_ind：col_ind[i] 就是分配给 robot i 的任务编号。
    row_ind, col_ind = linear_sum_assignment(C)

    return col_ind, C


def print_assignment(robots, tasks, col_ind, C):
    """打印分配结果，方便调试时确认算法是否真的在"交叉"分配。"""
    total_cost = 0.0
    rospy.loginfo("[hungarian] ===== 匈牙利算法分配结果 =====")
    rospy.loginfo("[hungarian] %-10s  %-6s  %-20s  %s",
                  "机器人", "任务#", "目标(x, y)", "代价(m)")
    for i, robot in enumerate(robots):
        j = col_ind[i]
        task = tasks[j]
        cost = C[i][j]
        total_cost += cost
        rospy.loginfo("[hungarian] %-10s  #%-5d  (%-5.2f, %-5.2f)        %.3f",
                      robot['namespace'], j,
                      float(task['x']), float(task['y']), cost)
    rospy.loginfo("[hungarian] 总代价: %.3f m", total_cost)
    rospy.loginfo("[hungarian] ================================")


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------

def main():
    rospy.init_node('task_assigner_hungarian')

    robots  = rospy.get_param('~robots', [])
    tasks   = rospy.get_param('~tasks', [])
    timeout = rospy.get_param('~goal_timeout', 180.0)

    # 参数校验
    if not robots:
        rospy.logwarn("[hungarian] 参数 ~robots 为空，退出。")
        return
    if not tasks:
        rospy.logwarn("[hungarian] 参数 ~tasks 为空，退出。")
        return
    if len(robots) != len(tasks):
        rospy.logerr("[hungarian] robots 数量(%d) != tasks 数量(%d)，"
                     "当前只支持 n==m 的方阵分配，退出。",
                     len(robots), len(tasks))
        return

    rospy.loginfo("[hungarian] %d 台机器人，%d 个任务，开始求解最优分配...",
                  len(robots), len(tasks))

    # ---- 步骤 1：匈牙利算法求最优分配 ----
    col_ind, C = solve_assignment(robots, tasks)
    print_assignment(robots, tasks, col_ind, C)

    # ---- 步骤 2：并发向每台机器人发送其分配到的目标 ----
    results = [None] * len(robots)
    threads = []

    for i, robot in enumerate(robots):
        ns       = robot['namespace']
        task_idx = col_ind[i]        # 匈牙利算法给 robot i 分配的任务编号
        goal_cfg = tasks[task_idx]   # 对应的目标点字典 {x, y, yaw}

        t = threading.Thread(
            target=dispatch_robot,
            args=(ns, goal_cfg, timeout, results, i),
            name='dispatch_{}'.format(ns)
        )
        t.daemon = True
        threads.append(t)

    # 同时启动所有线程（并发出发）
    for t in threads:
        t.start()

    # 等待所有线程完成
    for t in threads:
        t.join()

    # ---- 步骤 3：汇总结果 ----
    rospy.loginfo("[hungarian] ===== 所有机器人执行完毕 =====")
    for i, robot in enumerate(robots):
        rospy.loginfo("[hungarian]   %-10s  %s", robot['namespace'], results[i])


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass