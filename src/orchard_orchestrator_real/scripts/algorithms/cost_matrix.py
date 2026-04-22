#!/usr/bin/env python3
"""
cost_matrix.py — 代价矩阵构建

纯 Python：不 import rospy，可被单元测试或非 ROS 脚本调用。

当前实现：欧氏直线距离。
未来：Phase 2 可替换为 move_base 的 make_plan 服务返回的真实路径长度，
      接口签名不变（仍是 robot_positions / task_positions → ndarray）。
"""

import math

import numpy as np


def euclidean(p1, p2):
    """两个 (x, y) 点的欧氏距离。"""
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def build_cost_matrix(robot_positions, task_positions,
                      task_ids=None, wait_rounds=None, alpha=2.5):
    """
    构建 M × N 代价矩阵。

    参数：
      robot_positions: [(x, y), ...] 长度 M
      task_positions:  [(x, y), ...] 长度 N
      task_ids:        可选，[id, ...] 长度 N；列 j 的身份，用于查 wait_rounds
      wait_rounds:     可选，{id: 等待轮数}；None 或空 dict 时退化为纯欧氏距离
      alpha:           每轮等待的代价减免幅度（默认 2.5）

    返回：
      C: np.ndarray，shape (M, N)
         C[i, j] = euclidean(robot_i, task_j) - alpha * wait_rounds.get(task_ids[j], 0)
    """
    m = len(robot_positions)
    n = len(task_positions)
    C = np.zeros((m, n), dtype=float)
    for i, rp in enumerate(robot_positions):
        for j, tp in enumerate(task_positions):
            d = euclidean(rp, tp)
            penalty = 0.0
            if wait_rounds and task_ids is not None:
                penalty = alpha * wait_rounds.get(task_ids[j], 0)
            C[i, j] = d - penalty
    return C


if __name__ == "__main__":
    # 简单自检：3×3 对角矩阵应得到 0 代价
    robots = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    tasks = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    C = build_cost_matrix(robots, tasks)
    print("cost_matrix (对角) =\n", C)
    assert abs(C.trace()) < 1e-9
    print("[selftest] cost_matrix OK")
