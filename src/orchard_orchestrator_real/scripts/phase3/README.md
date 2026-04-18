# Phase 3 —— 多车匈牙利任务分配（占位）

## 目标

6 台 W4A 全部就绪后，中心调度节点跑在 VM，统一管理：
1. 从 TF 读各车位置（`map → robotN/base_footprint`）
2. 调用 `algorithms/cost_matrix.build_cost_matrix` 构建 M×N 代价
3. 调用 `algorithms/hungarian.assign_tasks` 求最优一对一分配
4. 为每台车独立启动线程调用 `utils/move_base_client.MoveBaseClient('/robotN/move_base').go_to(...)`
5. 单车失败不影响其他车（参考仿真版 `task_assigner_hungarian.py::dispatch_robot`）

## 从仿真版迁移的关键差异

| 维度 | 仿真版 `orchard_orchestrator/scripts/` | Phase 3 实机版 |
|-----|---|---|
| 定位源 | `/robotN/amcl_pose` (`PoseWithCovarianceStamped`) | TF `map → robotN/base_footprint` |
| 命名空间 | `robot1 / robot2 / ... / robot6` | W4A 实际配置的 namespace（待定） |
| 代价 | 欧氏直线 | 欧氏 → make_plan（Phase 2 已升级） |
| 果筐状态 | `ready_boxes` 内存字典 | 一致，多阶段串行共享 |
| 阶段 | A1 / B1 / A2 / B2 串行 | 一致，可能根据果园实机布局调整 |

## 新增文件（计划）

```
scripts/phase3/
├── multi_robot_orchestrator.py       # 中心调度主入口，迁移自仿真 orchestrator.py
├── dispatch_worker.py                # 单车工作线程，迁移自 task_assigner_hungarian.dispatch_robot
└── tf_multi_reader.py                # 封装"从 TF 拿 6 车位置" → dict[ns] = (x, y)

launch/multi_robot_orchestrator.launch
  参数：
    zones_yaml          config/zones_multi.yaml（含 6 车 namespace + depot + 任务清单）
    stages              A1,B1,A2,B2
    per_stage_timeout   120.0
```

## 可直接复用

- `algorithms/cost_matrix.py`（无须改动）
- `algorithms/hungarian.py`（无须改动；`print_assignment_table` 在分配日志上原地可用）
- `utils/move_base_client.py`（构造参数传 `/robotN/move_base` 即可）
- `utils/tf_helper.py`（构造参数传 `robotN/base_footprint`）

## 仿真侧参考文件（只读，不拷贝 rospy 耦合逻辑）

- `orchard_orchestrator/scripts/orchestrator.py` —— 阶段驱动 + 线程并发模式
- `orchard_task_assignment/scripts/task_assigner_hungarian.py` —— `dispatch_robot` 线程体模板

## 已知坑（仿真侧踩过）

- **B 阶段多车抢同一 depot cell → 全员 ABORTED(4)**：每台 transporter 必须配独立 `depot_slot`，不能共用 depot。
- **picker 到行尾后不让位 → transporter pickup 失败**：picker 完成 "采摘" 后要先移到 `stash` 点再登记 ready_box。
- 这些机制在 `orchard_orchestrator/CLAUDE.md` 里的"已修过的坑"章节有完整说明。
