#!/usr/bin/env python3
"""
hungarian.py — 匈牙利算法任务分配

纯 Python：不 import rospy，可 pytest / 单独运行。

功能：
  - assign_tasks(C): 输入代价矩阵，输出最优 (robot_idx, task_idx) 配对
  - print_assignment_table: 对齐打印分配结果（用 print，非 rospy）

从仿真版 orchard_orchestrator/scripts/hungarian_utils.py 迁移整理：
  - 拆掉 `hungarian_assign` 那种耦合了 id 列表的接口
  - 分配层只关心矩阵与索引；id / 名字由上层注入到 print_assignment_table
"""

import sys
import os

import numpy as np
from scipy.optimize import linear_sum_assignment

# 本模块同级：cost_matrix（供 __main__ 自检用，不是 assign_tasks 的依赖）
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)


def assign_tasks(cost_matrix):
    """
    用 Jonker-Volgenant 求最优 one-to-one 分配。

    参数：
      cost_matrix: np.ndarray, shape (M, N)

    返回：
      pairs: [(robot_idx, task_idx), ...]，按 robot_idx 升序
      M != N 时 scipy 返回 min(M, N) 对
    """
    C = np.asarray(cost_matrix, dtype=float)
    row_ind, col_ind = linear_sum_assignment(C)
    return list(zip(row_ind.tolist(), col_ind.tolist()))


def total_cost(pairs, cost_matrix):
    """给定配对与代价矩阵，返回总代价。"""
    C = np.asarray(cost_matrix, dtype=float)
    return float(sum(C[r, t] for r, t in pairs))


def print_assignment_table(pairs, cost_matrix, robot_ids=None, task_ids=None, title=""):
    """
    按仿真版 hungarian_utils 的对齐风格打印。

    用 print 而非 rospy.loginfo，保证无 rosmaster 也能跑（__main__ 自检友好）。
    """
    C = np.asarray(cost_matrix, dtype=float)
    print("\n[hungarian] ===== {} =====".format(title))
    print("[hungarian] {:<12}{:<22}{}".format("机器人", "任务", "代价(m)"))
    tot = 0.0
    for r, t in pairs:
        r_name = robot_ids[r] if robot_ids else "r{}".format(r)
        t_name = task_ids[t] if task_ids else "t{}".format(t)
        c = float(C[r, t])
        tot += c
        print("[hungarian] {:<12}{:<22}{:.3f}".format(r_name, t_name, c))
    print("[hungarian] 总代价: {:.3f} m".format(tot))
    print("[hungarian] ========================================")


if __name__ == "__main__":
    # ---- 3×3 通用自检 ----
    from cost_matrix import build_cost_matrix

    # 案例 1：对角放置 → 最优分配代价 0
    robots = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    tasks_same = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    C1 = build_cost_matrix(robots, tasks_same)
    pairs1 = assign_tasks(C1)
    print_assignment_table(pairs1, C1, title="自检 1：同位，预期 total=0")
    assert abs(total_cost(pairs1, C1)) < 1e-9, "自检 1 失败"

    # 案例 2：故意错位，验证算法真的在"交叉分配"而不是简单按顺序
    # 排列 tasks 为 (2,0)-(0,0)-(1,0)，最优应把 r0→t1(0,0), r1→t2(1,0), r2→t0(2,0)
    # 即 pairs = [(0,1), (1,2), (2,0)]，total = 0
    robots2 = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    tasks_perm = [(2.0, 0.0), (0.0, 0.0), (1.0, 0.0)]
    C2 = build_cost_matrix(robots2, tasks_perm)
    pairs2 = assign_tasks(C2)
    print_assignment_table(pairs2, C2, title="自检 2：错位置换，预期 total=0")
    assert abs(total_cost(pairs2, C2)) < 1e-9, "自检 2 失败：算法未做交叉分配"
    # 显式验证交叉
    mapping = dict(pairs2)
    assert mapping[0] == 1 and mapping[1] == 2 and mapping[2] == 0, \
        "自检 2 失败：期望 r0→t1, r1→t2, r2→t0，实得 {}".format(pairs2)

    # 案例 3：非零代价，验证"最小总代价"性质
    # robots=(0,0),(5,0); tasks=(5,0),(0,0)
    # 朴素分配(r0→t0, r1→t1) = 5+5 = 10；最优(r0→t1, r1→t0) = 0+0 = 0
    robots3 = [(0.0, 0.0), (5.0, 0.0)]
    tasks3 = [(5.0, 0.0), (0.0, 0.0)]
    C3 = build_cost_matrix(robots3, tasks3)
    pairs3 = assign_tasks(C3)
    print_assignment_table(pairs3, C3, title="自检 3：2×2 交叉，预期 total=0（非 10）")
    assert abs(total_cost(pairs3, C3)) < 1e-9, "自检 3 失败：未找到最优解"

    print("\n[selftest] OK: 三组自检全部通过，算法与代价矩阵接口就位。")
