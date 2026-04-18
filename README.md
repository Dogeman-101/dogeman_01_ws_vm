# 果园机器人自主导航仿真

基于 ROS Noetic + Gazebo 11 的麦克纳姆轮全向机器人果园巡检仿真。

6 台机器人在虚拟果园中**同时自主规划路径、自动避障、并发完成巡检**，全程无需人工干预。

---

## 这个项目是干什么的？

想象一个 12m × 8m 的小型果园，里面有三排果树形成的隔墙，机器人需要穿梭在各条走廊之间完成检查任务。

```
y=8  ┌────────────────────────────────┐
     │ →→→  robot1  Corridor 1  →→→  │
y=6  ├━━━━━━━━━━━━━━━━━━━┤  Row 1   │
     │ →→→  robot2  Corridor 2  →→→  │
y=4  ├━━━━━━━━━━━━━━━━━━━┤  Row 2   │
     │ →→→  robot3  Corridor 3  →→→  │
y=2  ├━━━━━━━━━━━━━━━━━━━┤  Row 3   │
     │ →→→  robot4  Corridor 4  →→→  │
y=0  └────────────────────────────────┘
     x=0                            x=12
  ↕ robot5（西侧通道）   robot6↕（东侧通道）
```

项目实现了：
- **6 台机器人同时出发**，各自走自己的路线，互不干扰
- 每台机器人独立进行**实时定位**（知道自己在地图上的位置）
- 遇到障碍物自动**重新规划路径**绕过去
- 任务完成后在终端打印每台机器人的执行结果
- 在导航层之上还叠了一个**四阶段采摘-运输流水线**：3 台机器人扮演 picker、3 台扮演 transporter，按 A1→B1→A2→B2 串行完成"北排采摘→运回 depot→南排采摘→运回 depot"的完整工作流（见后文 `orchard_orchestrator` 包）

**为什么用麦克纳姆轮？** 因为果园侧边通道很窄（约 1.35m），普通差速机器人需要原地转圈才能拐进去；麦克纳姆轮支持横向平移，能更优雅地通过窄道。

---

## 技术组成：自研 vs 第三方

项目的各个功能分别由"自己写的"和"调用现成工具包"两部分组成。

### 自研部分

| 是什么 | 文件位置 | 负责什么 |
|--------|----------|----------|
| 机器人 3D 模型 | `mecanum_robot/urdf/mecanum_robot.urdf.xacro` | 定义机器人的外形、尺寸、质量、四个车轮、激光雷达的位置 |
| 机器人外形参数 | `mecanum_robot/param/costmap_common_params_mecanum.yaml` | 告诉导航系统机器人有多"胖"，障碍物要留多大余量 |
| 定位参数 | `mecanum_robot/param/amcl_params.yaml` | 调整定位算法的精度和响应速度（全向轮专用配置） |
| 路径规划参数 | `mecanum_robot/param/dwa_local_planner_params_mecanum.yaml` | 调整机器人行走速度、刹车距离、是否允许横向移动等 |
| 果园仿真场景 | `orchard_navigation/worlds/orchard.world` | 描述 Gazebo 里的果园：地面、墙壁、果树排列 |
| 导航地图 | `orchard_navigation/maps/orchard_map.*` | 供机器人定位和规划路径用的平面栅格地图 |
| 多机器人启动文件 | `orchard_navigation/launch/multi_robot_navigation.launch` | 一键启动仿真环境 + 6 台机器人的全套导航系统 |
| 单机器人启动文件 | `orchard_navigation/launch/orchard_navigation.launch` | 调试用，只启动 1 台机器人 |
| RViz 配置 | `orchard_navigation/rviz/navigation.rviz` | 预设好的可视化布局（地图、激光点云、规划路径） |
| 任务列表（静态版） | `orchard_task_assignment/config/orchard_tasks.yaml` | 6 台机器人各自的目标坐标，改这里就能换路线 |
| 任务列表（匈牙利版） | `orchard_task_assignment/config/orchard_tasks_hungarian.yaml` | 机器人出发坐标 + 目标列表，由算法自动决定谁去哪 |
| 任务启动文件 | `orchard_task_assignment/launch/task_assignment.launch` | 启动静态版任务分配节点 |
| 任务启动文件（匈牙利版） | `orchard_task_assignment/launch/task_assignment_hungarian.launch` | 启动匈牙利版任务分配节点 |
| 任务分配程序（静态版） | `orchard_task_assignment/scripts/task_assigner.py` | Python 程序，按 yaml 配置同时向 6 台机器人下发固定目标 |
| 任务分配程序（匈牙利版） | `orchard_task_assignment/scripts/task_assigner_hungarian.py` | 用匈牙利算法自动求最短总路程的最优分配，再并发下发 |
| 匈牙利版说明文档 | `orchard_task_assignment/docs/hungarian_usage.md` | 详细介绍算法原理和使用方式 |
| 编排流水线配置 | `orchard_orchestrator/config/orchard_layout.yaml` | depot、6 个采摘任务（含让位点 stash）、picker/transporter 分组、各阶段时间参数 |
| 编排启动文件 | `orchard_orchestrator/launch/orchestrator.launch` | 加载上面的 yaml 并启动编排节点 |
| 4 阶段编排器 | `orchard_orchestrator/scripts/orchestrator.py` | 按 A1→B1→A2→B2 串行调度，每阶段单独跑一次匈牙利分配 |
| 匈牙利工具库 | `orchard_orchestrator/scripts/hungarian_utils.py` | 从匈牙利版任务分配器抽出的纯函数，供 orchestrator 复用；附带离线自检 |
| 动捕定位节点 | `mocap_localization/scripts/mocap_to_tf.py` | 订阅动捕 PoseStamped，计算并广播 map→odom TF，替代 AMCL |
| 动捕定位启动文件 | `mocap_localization/launch/mocap_to_tf.launch` | 启动 mocap_to_tf 节点，可配置动捕话题和机器人命名空间 |
| 仿真假动捕 relay | `mocap_localization/scripts/odom_to_pose.py` | 将 Gazebo P3D 的 Odometry 转为 PoseStamped，对齐实机动捕话题 |
| 假动捕启动文件 | `mocap_localization/launch/gazebo_fake_mocap.launch` | 启动 odom_to_pose relay，remap 话题名对齐 `/mocap_node/Robot_N/pose` |
| mocap 版多机启动文件 | `orchard_navigation/launch/multi_robot_navigation_mocap.launch` | 用 mocap_to_tf 替代 AMCL 的 6 机器人仿真（端到端） |
| mocap 版单车子 launch | `orchard_navigation/launch/single_robot_mocap.launch` | 单台车的 spawn + mocap 定位 + move_base，供主 launch 调用 |

### 调用的第三方工具包

| 工具包 | 干了什么 |
|--------|----------|
| **Gazebo 11** | 物理仿真引擎，模拟重力、碰撞；内置驱动插件模拟全向底盘运动和激光雷达扫描 |
| **AMCL** | 粒子滤波定位：机器人扫一圈激光，和地图对比，推算出"我在哪里" |
| **move_base** | 导航总调度：收到目标坐标后，调用规划器算出路径，发送速度指令给底盘 |
| **NavFn** | 全局路径规划器（A* 算法），在整张地图上找一条从 A 到 B 的路 |
| **DWA 局部规划器** | 局部避障，实时根据激光数据微调速度，绕过突然出现的障碍 |
| **map_server** | 把地图文件加载进来，让所有机器人都能查询 |
| **robot_state_publisher** | 根据 URDF 模型，实时广播机器人各部件的空间位置（供 RViz 显示） |
| **turtlebot3_navigation** | 借用了其中 3 个 move_base 框架参数文件；其余导航参数均被本项目自定义文件覆盖 |

---

### `mocap_localization` 包——动捕定位（替代 AMCL）

#### `scripts/mocap_to_tf.py` — 动捕→TF 转换节点

订阅动作捕捉系统发布的 `geometry_msgs/PoseStamped`（机器人在 map 坐标系中的真值位姿），计算并广播 `map → robotN/odom` TF 变换，从而替代 AMCL 的定位角色。

核心数学：`T_map_odom = T_map_base × T_odom_base⁻¹`。其中 `T_odom_base` 由 Gazebo planar_move 插件已有的 TF 广播获取，`T_map_base` 来自动捕输入。节点以 50Hz 定时广播，即使动捕数据短暂中断也保持 TF 连续。

设计上同时支持仿真（接 Gazebo p3d 插件的假动捕）和真机（接 mocap_nokov 的真动捕），接口一致——都是 PoseStamped 话题。

#### `launch/mocap_to_tf.launch` — 动捕定位启动文件

启动单台机器人的 `mocap_to_tf` 节点，通过 arg 配置机器人命名空间和动捕话题。

#### `scripts/odom_to_pose.py` — 仿真假动捕 relay

Gazebo P3D 插件发布的是 `nav_msgs/Odometry`，而 `mocap_to_tf` 期望 `geometry_msgs/PoseStamped`。这个 relay 节点做消息类型转换，并通过 launch 文件 remap 将话题名映射到 `/mocap_node/Robot_N/pose`，使 `mocap_to_tf` 无需任何修改即可在仿真中运行。

#### `launch/gazebo_fake_mocap.launch` — 假动捕 relay 启动文件

启动 `odom_to_pose` relay 节点，参数为 `robot_namespace`（如 `robot1`）和 `rigid_body_name`（如 `Robot_1`），自动 remap 输入输出话题。

---

## 各文件详解

### `mecanum_robot` 包——机器人本体

#### `urdf/mecanum_robot.urdf.xacro` — 机器人模型定义

用 XML 描述机器人长什么样、各部件怎么连接：

- **底盘**：0.35m（长）× 0.25m（宽）× 0.08m（高）的长方体，重 3kg
- **四个麦轮**：半径 4cm，带旋转关节，仅作可视化（实际运动由 Gazebo 插件直接驱动）
- **激光雷达**：圆柱形传感器，安装在底盘顶部，扫描范围 360°、最远 3.5m
- **Gazebo 全向驱动插件**：订阅 `cmd_vel` 速度指令，直接操控物理引擎移动，同时发布 `odom` 里程计数据
- **Gazebo 激光插件**：模拟激光雷达扫描，发布到 `scan` 话题
- **多机器人支持**：通过 `robot_ns` 参数区分每台机器人，如 `robot_ns:=robot1` 则所有话题和坐标系都带 `robot1/` 前缀

#### `param/costmap_common_params_mecanum.yaml` — 机器人"体型"参数

告诉代价地图机器人有多大、要和障碍物保持多远距离：

- `robot_radius: 0.20`：把机器人近似成半径 20cm 的圆
- `inflation_radius: 0.25`：障碍物周围额外膨胀 25cm（两者之和 = 45cm，刚好能通过 1.35m 宽的侧边通道）
- `obstacle_range / raytrace_range`：激光雷达感知障碍物的有效距离

#### `param/amcl_params.yaml` — 定位算法参数

调整粒子滤波定位器（AMCL）的行为：

- `odom_model_type: omni`：全向轮里程计模型，比差速模型更准确
- `min_particles / max_particles`：粒子数量（500~3000），越多定位越准但越慢
- `update_min_d / update_min_a`：机器人移动多少才更新一次定位（避免静止时浪费计算）
- 坐标系 ID（base_frame、odom_frame 等）不写在这里，由 launch 文件按机器人编号注入

#### `param/dwa_local_planner_params_mecanum.yaml` — 局部规划器参数

控制机器人实际行走的速度和行为：

- `holonomic_robot: true`：**关键参数**，开启横向移动支持（关掉则无法侧移）
- `y_vels: [-0.3, -0.1, 0.1, 0.3]`：侧向速度候选值（m/s），DWA 从中采样评估最优
- `max_vel_x / max_vel_y`：前进最快 0.4m/s，横移最快 0.3m/s
- `xy_goal_tolerance: 0.10`：距目标点 10cm 内视为到达
- `sim_time: 1.5`：每次预测 1.5 秒内的轨迹来评估最优速度

---

### `orchard_navigation` 包——场景和启动

#### `worlds/orchard.world` — Gazebo 仿真场景

SDF 格式描述的虚拟果园：
- 12m × 8m 的围墙
- 三排"果树"（实为矩形墙体），划分出四条东西走廊和两条南北侧边通道
- 地面物理属性、光照和阴影设置

#### `maps/orchard_map.pgm` + `maps/orchard_map.yaml` — 导航地图

- `.pgm`：240×160 像素的灰度图，白色 = 可通行，黑色 = 障碍，每像素代表 5cm
- `.yaml`：描述地图的分辨率（0.05m/px）、原点坐标、黑白判断阈值
- 地图由 Python + numpy 程序化生成，与 `orchard.world` 的墙体位置精确对齐

#### `launch/multi_robot_navigation.launch` — 主启动文件（AMCL 版）

一次性启动整套仿真环境：
1. 启动 Gazebo，加载 `orchard.world`
2. 启动 `map_server`，加载导航地图（全局共享一份）
3. 为每台机器人（robot1~6）依次：
   - 解析 URDF，在对应起点生成机器人模型
   - 启动 `robot_state_publisher`（广播关节 TF）
   - 启动 `amcl`（粒子滤波定位，含关键 remap：`static_map → /static_map`）
   - 启动 `move_base`（导航栈，含关键 remap：`map → /map`）

#### `launch/multi_robot_navigation_mocap.launch` — 主启动文件（mocap 版）

与 AMCL 版功能相同，但用 `mocap_to_tf`（动捕/P3D 假动捕）替代 AMCL 做定位。结构更清晰：主 launch 只负责 Gazebo + map_server，6 台车各自通过 `single_robot_mocap.launch` 子文件启动。

每台车的节点栈：`spawn_model` → `robot_state_publisher` → `odom_to_pose`（P3D relay）→ `mocap_to_tf`（TF 广播）→ `move_base`（导航）。

两个版本可切换使用，不互相影响。

#### `launch/single_robot_mocap.launch` — 单车子 launch（mocap 版内部使用）

封装单台机器人的完整栈，接收 `robot_namespace`、`rigid_body_name`、`model_idx`、`x`、`y`、`yaw` 参数。由 `multi_robot_navigation_mocap.launch` 调用 6 次。

#### `launch/orchard_navigation.launch` — 单机器人调试文件

功能与 AMCL 版主启动文件相同，但只启动 1 台机器人，用于调试和开发。**不可与多机器人 launch 同时运行。**

#### `rviz/navigation.rviz` — RViz 可视化配置

预设好的 RViz 布局，保存了：
- 地图显示图层
- 激光扫描点云图层
- 机器人 3D 模型图层
- 全局/局部规划路径图层
- 粒子云（AMCL 定位粒子）图层

---

### `orchard_task_assignment` 包——任务分配

#### `config/orchard_tasks.yaml` — 任务配置文件

最常修改的文件，定义 6 台机器人各自的目标点：

```yaml
goal_timeout: 180.0       # 每台机器人最多等待多少秒（超时则放弃该机器人）
robots:
  - namespace: "robot1"
    goal: {x: 9.0, y: 7.0, yaw: 0.0}   # 目标坐标（米）和到达后的朝向（弧度）
  - namespace: "robot2"
    goal: {x: 9.0, y: 5.0, yaw: 0.0}
  # ...
```

直接编辑坐标即可换路线，**无需重新编译**。

#### `launch/task_assignment.launch` — 任务启动文件

启动 `task_assigner.py` 节点，并把上面的 yaml 文件作为参数加载进去。

#### `scripts/task_assigner.py` — 任务分配程序（核心逻辑）

Python 编写，运行逻辑如下：

1. 从参数服务器读取 `~robots` 列表（即 yaml 里配置的 6 台机器人）
2. 为每台机器人创建一个**独立线程**
3. 所有线程同时启动——6 台机器人**并发出发**
4. 每个线程：连接对应机器人的 `move_base` 动作服务器 → 发送目标坐标 → 等待完成
5. 主线程等待所有线程结束，汇总打印结果

与导航层的**唯一接口**是标准的 `move_base` action（`MoveBaseGoal`），因此任务分配逻辑可以随时替换，无需修改导航部分。

#### `scripts/task_assigner_hungarian.py` — 匈牙利算法版任务分配（进阶）

在静态版的基础上，增加了"自动决定谁去哪"的能力：

1. 从 `~robots` 读取每台机器人的**出发坐标**，从 `~tasks` 读取所有**目标点**
2. 构建一张 6×6 的"代价表"——机器人 i 到目标 j 的直线距离
3. 用**匈牙利算法**（`scipy.optimize.linear_sum_assignment`）找到让所有机器人总行驶距离最短的分配方案
4. 打印分配结果（谁去哪、各自多远、总代价），然后并发执行——后续流程与静态版完全相同

`config/orchard_tasks_hungarian.yaml` 里的任务点顺序是**故意打乱的**，用来验证算法真的在"交叉"分配（而不是简单地 robot1→任务1、robot2→任务2……）。预期总代价约 34m，顺序分配约 37m，节省约 9%。

---

### `orchard_orchestrator` 包——4 阶段采摘-运输编排

这是项目最上层的编排包。和 `orchard_task_assignment` 的**一次性派发**不同，它把 6 台机器人分成两个角色，按 4 个阶段**串行**执行完整的采摘-运输流水线：

```
A1：3 台 picker 从西侧入口进走廊，穿越 row_north 到东端，放下 3 个果筐
B1：3 台 transporter 从 depot 出发，到东端取货，运回 depot
A2：3 台 picker 就地掉头，反向穿越 row_south 回到西端，再放 3 个果筐
B2：3 台 transporter 再跑一趟，到西端取第二批，运回 depot
```

每个阶段都**独立跑一次匈牙利算法**做"谁去哪"的最优分配，分配关系是运行时动态决定的，不是写死在配置里。A2 的分配尤其有戏：picker 完成 A1 后停在东端不同走廊，此时再次分配到 row_south 通常会产生"交叉"——某台 picker 要跳到另一条走廊去，能看到匈牙利算法真的在干活。

#### `config/orchard_layout.yaml` — 整套流水线的"剧本"

定义 depot 位置、6 个采摘任务的 entry/exit/stash 坐标、picker/transporter 分组、各阶段的停留时长等。
- **entry / exit**：picker 穿越果树行的起点和终点。A1 由西向东（yaw=0），A2 由东向西（yaw=π）
- **stash**：picker 完成采摘后的"让位点"。没有它，picker 会停在 pickup 坐标上，导致 transporter 的 move_base 把 picker 当作终点障碍物、判 pickup 不可达（B 阶段会全员 state=4 ABORTED）。A1 stash 在东侧通道墙 y 值位置（`(11.5, 2/4/6)`，那里无墙），A2 stash 沿走廊退 1.5m 进果树行内部（`(2.0, 1/3/5)`）
- **picker_robots / transporter_robots**：把 robot1~6 按角色分成两组；namespace 必须和 `multi_robot_navigation.launch` 里实际 spawn 出来的机器人一致
- **mode.pipeline**：保持 `false`。`true` 分支目前会抛 `NotImplementedError`，串行模式是唯一支持的模式

直接编辑 yaml 就能换任务、调时长，**不需要重新编译**。

#### `launch/orchestrator.launch` — 编排启动文件

启动 `orchestrator.py` 节点并把 `orchard_layout.yaml` 全部加载到参数服务器。

#### `scripts/orchestrator.py` — 4 阶段驱动核心

整个编排器就一个 `Orchestrator` 类，主循环非常清晰：

1. 启动时订阅 6 台机器人的 `amcl_pose`（实时位置，作为匈牙利算法的代价输入）
2. 依次连接 6 台机器人的 `move_base` action server（任何一台连不上就直接退出）
3. 等 3 秒让 `amcl_pose` 首帧到达，避免用 fallback 坐标污染首轮分配
4. 按 A1 → B1 → A2 → B2 顺序调用 `execute_picking_round` / `execute_transport_round`
5. 每个阶段：跑一次匈牙利分配 → 为每台机器人开一个线程并发下发 → join 等全员结束
6. 打印每个阶段耗时和总耗时

picker 的每个任务由 `_run_picker_task` 执行：**进 entry → 穿到 exit → 原地"采摘" 5s → 退到 stash 让位 → 登记果筐到 `ready_boxes`**。transporter 的任务由 `_run_transporter_task` 执行：**到 pickup → 装载 3s → 回 depot → 卸货 3s**。

阶段之间通过 `self.ready_boxes` 字典传递"某某果筐已就绪"的信号。B 阶段只处理前一轮 A 实际登记成功的果筐——即使某个 picker 半路失败，B 阶段也会跳过缺失的任务、只调度剩下的 transporter，不会崩溃。

#### `scripts/hungarian_utils.py` — 可复用的匈牙利工具

从 `task_assigner_hungarian.py` 抽出的核心代价-分配逻辑，封装成三个纯函数：
- `euclidean(p1, p2)`：两点直线距离
- `hungarian_assign(robot_positions, task_targets, ...)`：构造代价矩阵 + 调用 `scipy.optimize.linear_sum_assignment` 求最优分配，返回 `{robot_id: task_id}` 字典
- `print_assignment_table(...)`：按统一格式打印分配结果

代价函数目前是**欧氏直线距离**，不是 move_base 真实路径长度——这是有意的简化，升级为 `make_plan` 服务是已知改进方向。

这个文件自带一个小自检（`if __name__ == "__main__"`），直接 `python3` 就能跑，**不需要 rosmaster**：
```bash
python3 src/orchard_orchestrator/scripts/hungarian_utils.py
```
预期输出：A1 对角分配总代价为 0；A2 `robot3` 被分配到 `row1_south` 承担 6m 最长跨度（这是算法真的在"交叉"分配的证据）。如果这个自检挂了，说明算法实现或坐标约定已经偏离设计，先修它再动 orchestrator。

---

> **CMakeLists.txt 和 package.xml**：每个包里都有这两个文件，是 ROS 工程的标准配置。`package.xml` 声明包名和依赖，`CMakeLists.txt` 告诉编译器要编译什么。本项目的 Python 脚本不需要编译，C++ 代码也没有，这两个文件只起"登记注册"的作用。

---

## 环境要求

| 软件 | 版本 |
|------|------|
| Ubuntu | 20.04 |
| ROS | Noetic |
| Gazebo | 11（随 ROS Noetic Desktop 自动安装） |

还不知道怎么安装 ROS？参考[官方安装文档](http://wiki.ros.org/noetic/Installation/Ubuntu)，按照 Desktop-Full 安装即可。

安装好 ROS 后，还需要以下额外的包：

```bash
# ROS 相关包
sudo apt install -y \
  ros-noetic-navigation \
  ros-noetic-turtlebot3-navigation \
  ros-noetic-gazebo-ros-pkgs \
  ros-noetic-robot-state-publisher \
  ros-noetic-map-server \
  ros-noetic-amcl \
  ros-noetic-move-base

# 匈牙利算法版额外依赖（仅使用匈牙利版时需要）
sudo apt install -y python3-scipy python3-numpy
```

---

## 快速上手

### 第一步：编译工作空间

```bash
cd ~/dogeman_01_ws
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
```

> 建议把最后一行加到 `~/.bashrc`，这样每次开终端都自动生效：
> `echo "source ~/dogeman_01_ws/devel/setup.bash" >> ~/.bashrc`

### 第二步：启动仿真（终端 A）

```bash
roslaunch orchard_navigation multi_robot_navigation.launch
```

等待 20~30 秒，Gazebo 窗口会出现 6 台机器人分布在果园各起点。

### 第三步：下发任务（终端 B）

```bash
source ~/dogeman_01_ws/devel/setup.bash
roslaunch orchard_task_assignment task_assignment.launch
```

6 台机器人同时开始向各自目标点行进。终端会实时输出状态，全部完成后汇总打印：

```
[task_assigner]   robot1      SUCCEEDED
[task_assigner]   robot2      SUCCEEDED
[task_assigner]   robot3      SUCCEEDED
[task_assigner]   robot4      SUCCEEDED
[task_assigner]   robot5      SUCCEEDED
[task_assigner]   robot6      SUCCEEDED
```

### 可选：使用 mocap 版仿真（用 P3D 假动捕替代 AMCL）

终端 A 改用 mocap 版启动文件，其余步骤不变：

```bash
roslaunch orchard_navigation multi_robot_navigation_mocap.launch
```

数据流：Gazebo P3D 插件 → `odom_to_pose` relay → `mocap_to_tf` → `map→odom` TF → `move_base`。定位精度等同 ground truth，无粒子滤波收敛延迟。后续的任务分配和编排 launch 无需任何修改，直接在终端 B 启动即可。

### 可选：使用匈牙利算法版（让程序自动决定谁去哪）

终端 A 保持不变（仿真继续运行），终端 B 改用：

```bash
roslaunch orchard_task_assignment task_assignment_hungarian.launch
```

节点启动时会先打印算法计算出的分配结果，例如：

```
[hungarian] ===== 匈牙利算法分配结果 =====
[hungarian] 机器人       任务#  目标(x, y)            代价(m)
[hungarian] robot1      #2     (9.00, 7.00)          5.500
[hungarian] robot2      #4     (9.00, 5.00)          5.500
[hungarian] robot3      #5     (9.00, 3.00)          5.500
[hungarian] robot4      #0     (9.00, 1.00)          5.500
[hungarian] robot5      #3     (0.75, 1.00)          6.000
[hungarian] robot6      #1     (11.25, 7.00)         6.000
[hungarian] 总代价: 34.000 m
[hungarian] ================================
```

注意 robot1 被分配到了 `tasks[2]` 而非 `tasks[0]`——这正是匈牙利算法在工作，而不是简单地按顺序分配。

### 可选：运行 4 阶段采摘-运输编排

终端 A 保持不变（仿真继续运行），终端 B 改用：

```bash
roslaunch orchard_orchestrator orchestrator.launch
```

依赖：`python3-scipy` + `python3-numpy`（系统 apt 包，如果已装过匈牙利版就不用再装）。

编排器会按下面的节奏推进，全程约 5~8 分钟：

```
阶段 A1：3 台 picker 从西到东穿过 row_north，在东端放下 3 个果筐
阶段 B1：3 台 transporter 去东端取货，运回 depot 卸货
阶段 A2：3 台 picker 掉头，从东到西穿过 row_south，在西端放下第二批果筐
阶段 B2：3 台 transporter 再去西端取货，运回 depot
```

每个阶段启动前会打印匈牙利算法的分配表，例如：
```
[hungarian] ===== 阶段 A2 采摘分配 =====
[hungarian] 机器人         任务                    代价(m)
[hungarian] robot1      row1_south            1.000
[hungarian] robot2      row2_south            1.000
[hungarian] robot3      row3_south            1.000
[hungarian] 总代价: 3.000 m
```

全部完成后会汇总打印 4 个阶段的耗时和总耗时。任务完成编排节点自动退出（不是持久服务），仿真本体继续运行，可以直接再启动下一轮。

### 离线跑匈牙利自检（不需要启动仿真）

如果想验证匈牙利算法实现有没有跑偏，可以单独跑这个自检：
```bash
python3 src/orchard_orchestrator/scripts/hungarian_utils.py
```
这个脚本不依赖 rosmaster 和 Gazebo，几毫秒跑完，专门用来在改动 orchestrator 之前先确认底层分配函数没问题。

---

## 常见问题

**Q：启动报错 "Address already in use" 或 Gazebo 闪退（exit 255）**

上次没有正常关闭，有残留进程。执行以下命令清理：
```bash
pkill -f gzserver; pkill -f gzclient; pkill -f rosmaster
```

**Q：某台机器人报 "move_base not available after 30s"**

导航栈还没启动好，task_assigner 等了 30 秒超时了。确认终端 A 的仿真已经稳定（没有持续报错）后再执行终端 B。

**Q：机器人卡在原地不动（AMCL 版）**

AMCL 定位可能漂移了。打开 RViz，点击工具栏的 **2D Pose Estimate**，在地图上机器人实际位置点击并拖出朝向箭头，等粒子云收敛即可恢复。如果频繁出现定位漂移，可以改用 mocap 版启动文件（`multi_robot_navigation_mocap.launch`），定位精度等同 ground truth，不存在粒子收敛问题。

**Q：机器人卡在墙边过不去**

检查 `costmap_common_params_mecanum.yaml` 里的 `inflation_radius`，当前为 0.25m（临界值）。不要随意增大，否则侧边通道会被判定为不可通行。

**Q：运行匈牙利版报 `No module named 'scipy'`**

```bash
sudo apt install -y python3-scipy python3-numpy
```

安装完后无需重新编译，直接重新运行 `roslaunch` 即可。

**Q：跑 orchestrator 时 B 阶段所有 transporter 都报 `目标失败 state=4`**

picker 采摘完停在了 pickup 坐标上，transporter 的 move_base 把 picker 当作终点障碍物，判 pickup 不可达。确认 `orchard_layout.yaml` 里**每个 task 都配置了 `stash` 字段**——picker 会在"采摘"完后先退到 stash 让位，再登记果筐。如果你删了 stash 或者 stash 点本身就落在别的机器人附近，这个症状就会复现。

**Q：orchestrator 某阶段 `成功=False`，但进程没有崩，会继续跑下一阶段**

这是故意的：阶段失败不会拖累后续阶段，B 阶段只会处理实际登记成功的果筐，缺失的任务会在日志里标为 `缺失果筐 [...]`。如果想排查为什么失败，去翻前面的 `目标失败 state=N` 或 `进入 entry 失败` 日志。

---

## 当前构建范围（2026-04-18）

VM 侧的 `~/dogeman_01_ws` 目前**只构建实机栈**，上文的仿真四个包通过 `catkin config --skiplist` 从构建列表里排除：

| 状态 | 包 |
|------|-----|
| 参与 `catkin build` | `orchard_map`、`mocap_localization` |
| skiplist（文件与 git 历史保留，不编译） | `mecanum_robot`、`orchard_navigation`、`orchard_orchestrator`、`orchard_task_assignment` |

VM 上日常启动命令：

```bash
roslaunch orchard_map vm_master.launch
```

它一条命令起动捕驱动（`mocap_nokov`）+ 地图 + rviz。前提是先 `source ~/catkin_mocap/devel/setup.bash` 和 `source ~/dogeman_01_ws/devel/setup.bash`。详细使用方式见 [src/orchard_map/README.md](src/orchard_map/README.md)。

### 恢复仿真能力

如果需要重新跑本文档上面各小节描述的 Gazebo 仿真流程：

```bash
cd ~/dogeman_01_ws
catkin config --no-skiplist
catkin build
```

之后上文所有 `roslaunch orchard_navigation …` / `roslaunch orchard_task_assignment …` / `roslaunch orchard_orchestrator …` 命令就恢复可用。

### 查看 / 重设 skiplist

```bash
catkin config | grep -A 2 "Skiplisted"        # 看当前 skiplist
catkin config --skiplist mecanum_robot orchard_navigation orchard_orchestrator orchard_task_assignment
```
