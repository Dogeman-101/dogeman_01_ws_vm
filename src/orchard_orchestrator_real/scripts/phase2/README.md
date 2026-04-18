# Phase 2 —— yaml 驱动的航点执行（占位）

## 目标

让 Phase 1 的 `waypoint_runner.py` 升级为：
- 从 `config/zones.yaml` 读 `waypoints` 字典 + `task_sequence` 列表，不再硬编码坐标
- 支持 `loop` 模式（连走 N 圈或无限循环）
- 代价估计从欧氏距离升级为 move_base 的 `make_plan` 服务返回的真实路径长度
  - 这一步只改 `algorithms/cost_matrix.py::build_cost_matrix` 的实现，
    接口不变，Phase 3 的分配层直接复用

## 新增文件（计划）

```
scripts/phase2/
├── waypoint_yaml_runner.py         # 读 zones.yaml，循环执行
└── make_plan_cost.py               # 调用 /move_base/make_plan，返回路径长度
```

## launch（计划）

```
launch/waypoint_yaml.launch
  参数：
    zones_yaml      default "$(find orchard_orchestrator_real)/config/zones.yaml"
    loop            default false
    max_rounds      default 1
    cost_source     default "euclidean"   # euclidean | make_plan
```

## 不改动的接口

- `algorithms/cost_matrix.py::build_cost_matrix(robots, tasks) → ndarray`
- `algorithms/hungarian.py::assign_tasks(C) → [(r, t), ...]`
- `utils/move_base_client.py::MoveBaseClient.go_to(x, y, yaw, timeout)`
- `utils/tf_helper.py::TFHelper.get_pose() → (x, y, yaw)`

## 验证思路

先用 euclidean cost 跑通 yaml 驱动；再把 cost_source 切到 make_plan，
对比两种代价下的航点总时长（预期 make_plan 更真实、绕障时差异明显）。
