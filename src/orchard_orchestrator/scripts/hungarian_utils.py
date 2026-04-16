#!/usr/bin/env python3
"""
hungarian_utils.py — 匈牙利算法分配工具

从 orchard_task_assignment/scripts/task_assigner_hungarian.py 抽取的核心
代价-分配逻辑，封装为纯函数便于在 orchestrator 中复用。

依赖：numpy + scipy（系统 apt 包，不进 package.xml）
"""

import math

import numpy as np
from scipy.optimize import linear_sum_assignment


def euclidean(p1, p2):
    """计算两个 (x, y) 点的欧氏距离。"""
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def hungarian_assign(robot_positions, task_targets, robot_ids, task_ids):
    """
    用匈牙利算法（Jonker-Volgenant 实现）求一对一最优分配。

    参数：
      robot_positions: [(x, y), ...] 长度 N
      task_targets:    [(x, y), ...] 长度 M
      robot_ids:       [str, ...]    长度 N，机器人命名空间
      task_ids:        [str, ...]    长度 M，任务 id

    当前实现要求 N == M（方阵分配），与 orchard 场景一致。

    返回：
      assignment:  dict, {robot_id: task_id}，由 scipy 求解出的最优对
      cost_matrix: np.ndarray, shape (N, M)，每格为欧氏距离
      total_cost:  float，最优分配的总代价

    代价函数层级：目前使用欧氏直线距离。后续可升级为调用
    move_base 的 make_plan 服务获得真实路径长度。
    """
    n = len(robot_positions)
    m = len(task_targets)
    if n != m:
        raise ValueError(
            "hungarian_assign currently requires a square matrix, got {}x{}".format(n, m)
        )
    if n != len(robot_ids) or m != len(task_ids):
        raise ValueError("robot_ids / task_ids length mismatch")

    cost = np.zeros((n, m), dtype=float)
    for i, rp in enumerate(robot_positions):
        for j, tt in enumerate(task_targets):
            cost[i, j] = euclidean(rp, tt)

    row_ind, col_ind = linear_sum_assignment(cost)
    assignment = {robot_ids[r]: task_ids[c] for r, c in zip(row_ind, col_ind)}
    total_cost = float(cost[row_ind, col_ind].sum())
    return assignment, cost, total_cost


def print_assignment_table(assignment, cost_matrix, robot_ids, task_ids, title=""):
    """
    按 task_assigner_hungarian.py 的风格对齐打印分配结果。
    用 print 而非 rospy.loginfo，便于在没有 rosmaster 的单元测试中使用。
    """
    print("\n[hungarian] ===== {} =====".format(title))
    print("[hungarian] {:<12}{:<22}{}".format("机器人", "任务", "代价(m)"))
    total = 0.0
    for r_id, t_id in assignment.items():
        r_idx = robot_ids.index(r_id)
        t_idx = task_ids.index(t_id)
        c = float(cost_matrix[r_idx, t_idx])
        total += c
        print("[hungarian] {:<12}{:<22}{:.3f}".format(r_id, t_id, c))
    print("[hungarian] 总代价: {:.3f} m".format(total))
    print("[hungarian] ========================================")


# 快速自检：手算 A1 / A2 两轮预期分配
if __name__ == "__main__":
    # --- A1: picker 从西侧 (0.5, y) 去 row_north entry (0.5, y) ---
    pickers_start = [(0.5, 3.0), (0.5, 5.0), (0.5, 7.0)]
    row_north_entry = [(0.5, 3.0), (0.5, 5.0), (0.5, 7.0)]
    a1, c1, tot1 = hungarian_assign(
        pickers_start, row_north_entry,
        ["robot1", "robot2", "robot3"],
        ["row1_north", "row2_north", "row3_north"],
    )
    print_assignment_table(a1, c1, ["robot1", "robot2", "robot3"],
                           ["row1_north", "row2_north", "row3_north"],
                           title="A1 预期（对角分配，总 cost = 0）")
    assert abs(tot1) < 1e-6, "A1 expected zero cost"

    # --- A2: picker 在 row_north exit (11.5, y) 去 row_south entry (11.5, y') ---
    pickers_after_a1 = [(11.5, 3.0), (11.5, 5.0), (11.5, 7.0)]
    row_south_entry = [(11.5, 1.0), (11.5, 3.0), (11.5, 5.0)]
    a2, c2, tot2 = hungarian_assign(
        pickers_after_a1, row_south_entry,
        ["robot1", "robot2", "robot3"],
        ["row1_south", "row2_south", "row3_south"],
    )
    print_assignment_table(a2, c2, ["robot1", "robot2", "robot3"],
                           ["row1_south", "row2_south", "row3_south"],
                           title="A2 预期（robot3 承担 6m 跨度）")
    assert abs(tot2 - 6.0) < 1e-6, "A2 expected total cost = 6.0 m"
    # robot3 应被分配到 row1_south（最长跨度）
    assert a2["robot3"] == "row1_south", "A2 non-trivial check failed"

    print("\n[selftest] OK: A1 total=0, A2 total=6, robot3 takes long jump.")
