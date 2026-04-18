# orchard_orchestrator_real

动捕下的实机（W4A + Nokov）任务编排节点骨架，按三阶段推进。

## 三阶段总览

| 阶段 | 范围 | 代码 | 状态 |
|-----|-----|-----|-----|
| Phase 1 | 单车单点诊断 + 单车航点序列 | `scripts/phase1/` | **本次实现** |
| Phase 2 | yaml 驱动的航点、循环模式、make_plan 路径代价 | `scripts/phase2/README.md` | 占位 |
| Phase 3 | 6 车匈牙利分配、多 namespace move_base、并发 dispatch | `scripts/phase3/README.md` | 占位 |

## 前置（每次启动都要）

| 终端 | 机器 | 命令 |
|-----|------|-----|
| 1 | VM | `roslaunch orchard_map vm_master.launch`（动捕 + 地图 + rviz） |
| 2 | W4A | `roslaunch w4a_bringup_nav.launch`（bringup + EKF + move_base） |

VM 先起 1，W4A 再起 2；等 `rosnode list` 里看到 `/move_base` 后才跑下面的调度 launch。

## Phase 1 使用

### 场景 A：单点诊断

排查"给正前方一个点，车往后跑"之类的坐标系/标定问题。

```bash
# 1. 改目标（三行常量）
vi src/orchard_orchestrator_real/scripts/phase1/single_goal_debug.py
#    GOAL_X / GOAL_Y / GOAL_YAW

# 2. 起 launch
source devel/setup.bash
roslaunch orchard_orchestrator_real single_goal_debug.launch
```

启动后每秒打印：
- 当前位姿（x, y, yaw 弧度+度数）
- 到目标的距离、绝对方位、需要转的角度、方向分类（前/后/左前/右前）
- `/cmd_vel`（linear.x, **linear.y**, angular.z）
- move_base 状态码 + 可读名

W4A 是麦轮车，`linear.y` 恒 0 = 规划器没启用 holonomic，**不是**本节点的 bug。

### 场景 B：航点序列

单点跑通后，给一串坐标让车按顺序走一圈。

```bash
# 1. 改航点（WAYPOINTS 列表）
vi src/orchard_orchestrator_real/scripts/phase1/waypoint_runner.py

# 2. 起 launch
source devel/setup.bash
roslaunch orchard_orchestrator_real waypoint_runner.launch
```

每航点失败重试 1 次再跳过，末尾打印成功/失败汇总。

## 依赖

| 包 | 来源 | 是否写入 package.xml |
|----|------|------|
| rospy / std_msgs / geometry_msgs / move_base_msgs / actionlib / tf | ROS Noetic | 是 |
| numpy / scipy | `sudo apt-get install python3-{numpy,scipy}` | 否（系统级） |

## 目录

```
orchard_orchestrator_real/
├── CMakeLists.txt / package.xml
├── README.md                           # 本文件
├── config/zones.yaml                   # Phase 2 用，Phase 1 不读
├── launch/
│   ├── single_goal_debug.launch
│   └── waypoint_runner.launch
└── scripts/
    ├── algorithms/                     # 纯 Python，无 rospy，可 pytest
    │   ├── cost_matrix.py              # build_cost_matrix(robots, tasks) → ndarray
    │   └── hungarian.py                # assign_tasks(C) → [(r,t),...]，带 __main__ 自检
    ├── utils/                          # ROS 交互封装，无业务逻辑
    │   ├── move_base_client.py
    │   └── tf_helper.py
    ├── phase1/                         # 本次实现
    │   ├── single_goal_debug.py
    │   └── waypoint_runner.py
    ├── phase2/README.md                # 计划
    └── phase3/README.md                # 计划
```

## 代码分层（严格）

- `algorithms/` 禁止 `import rospy / tf`
- `utils/` 禁止分配算法 / yaml 加载 / 业务逻辑
- `phase1/` 禁止直接构造 `SimpleActionClient` / `tf.TransformListener`，必须经 `utils/`

## 自检

```bash
# 算法层（不需要 roscore）
python3 src/orchard_orchestrator_real/scripts/algorithms/hungarian.py
# 预期：三组自检全部通过

# 编译
catkin build orchard_orchestrator_real

# launch 解析（不启动 ROS）
roslaunch --files orchard_orchestrator_real single_goal_debug.launch
roslaunch --files orchard_orchestrator_real waypoint_runner.launch
```
