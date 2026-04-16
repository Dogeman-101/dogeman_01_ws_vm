# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 环境配置

```bash
source /opt/ros/noetic/setup.bash
source ~/dogeman_01_ws/devel/setup.bash   # 首次构建后执行
```

无需设置 `TURTLEBOT3_MODEL` 环境变量，launch 文件不依赖该变量。

## 构建与运行

```bash
# 构建
cd ~/dogeman_01_ws && catkin_make

# 【主流程】步骤一：启动 6 机器人仿真（终端 A）
roslaunch orchard_navigation multi_robot_navigation.launch

# 【主流程】步骤二：仿真稳定后触发任务（终端 B）
# task_assigner.py 在所有机器人完成后自动退出，非持久服务
roslaunch orchard_task_assignment task_assignment.launch

# 【编排流程】步骤二替代方案：4 阶段采摘-运输编排（终端 B）
# 6 台机器人按 picker/transporter 角色跑 A1→B1→A2→B2 串行工作流，完成后退出
roslaunch orchard_orchestrator orchestrator.launch

# 【调试用】单机器人仿真（不含任务分配，勿与多机器人 launch 同时运行）
roslaunch orchard_navigation orchard_navigation.launch

# 验证 launch 文件节点列表（不启动 ROS）
roslaunch --nodes orchard_navigation multi_robot_navigation.launch

# 验证 URDF 解析
rosrun xacro xacro src/mecanum_robot/urdf/mecanum_robot.urdf.xacro
rosrun xacro xacro src/mecanum_robot/urdf/mecanum_robot.urdf.xacro robot_ns:=robot1

# 监控单台机器人导航状态
rostopic echo /robot1/move_base/status
```

**Gazebo 残留进程清理**（崩溃或强制关闭后必须执行，否则下次启动 exit 255）：
```bash
pkill -f gzserver; pkill -f gzclient; pkill -f rosmaster
```

**Gazebo 彻底重置**（sim_time 冻结、`pkill -INT` 后仍有 gzserver/roslaunch 残留时）：
```bash
pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f rosmaster; pkill -9 -f roslaunch
```

## 技术栈

| 层次 | 技术 |
|------|------|
| ROS 发行版 | ROS Noetic (Ubuntu 20.04) |
| 仿真器 | Gazebo 11 |
| 定位 | AMCL（omni 里程计模型，每台独立实例） |
| 路径规划 | move_base：NavFn 全局 + DWA 局部（每台独立） |
| 机器人驱动 | `libgazebo_ros_planar_move`（官方 gazebo_plugins，全向） |
| 传感器 | `libgazebo_ros_laser`（360° 激光，3.5m 量程，10Hz） |
| 任务分配 | `task_assigner.py` / `task_assigner_hungarian.py`（一次性派发） |
| 多阶段编排 | `orchard_orchestrator/orchestrator.py`（4 阶段串行 + 每阶段匈牙利分配） |

## 仓库结构与架构

```
src/
├── mecanum_robot/                   # 机器人描述 + 导航参数（可独立复用）
│   ├── urdf/mecanum_robot.urdf.xacro
│   └── param/
│       ├── costmap_common_params_mecanum.yaml
│       ├── dwa_local_planner_params_mecanum.yaml
│       └── amcl_params.yaml         # 多机器人 AMCL 参数（不含 frame_id）
├── orchard_navigation/              # 场景 + 地图 + 启动文件
│   ├── worlds/orchard.world
│   ├── maps/orchard_map.{pgm,yaml}  # 240×160px，0.05m/px，覆盖 12m×8m
│   ├── launch/
│   │   ├── multi_robot_navigation.launch   # 【主用】6 机器人
│   │   └── orchard_navigation.launch       # 单机器人调试用
│   └── rviz/navigation.rviz
├── orchard_task_assignment/         # 一次性任务分配层（依赖 move_base）
│   ├── config/
│   │   ├── orchard_tasks.yaml               # 静态派发版配置
│   │   └── orchard_tasks_hungarian.yaml     # 匈牙利版配置（含 start + tasks）
│   ├── docs/hungarian_usage.md              # 匈牙利版使用说明
│   ├── launch/
│   │   ├── task_assignment.launch
│   │   └── task_assignment_hungarian.launch
│   └── scripts/
│       ├── task_assigner.py                 # 静态派发版（保留作对比）
│       └── task_assigner_hungarian.py       # 匈牙利算法版
└── orchard_orchestrator/            # 4 阶段采摘-运输编排层（依赖 move_base）
    ├── config/orchard_layout.yaml           # depot + 6 任务 + picker/transporter 分组 + 时间参数
    ├── launch/orchestrator.launch
    └── scripts/
        ├── hungarian_utils.py               # 可复用的分配工具（euclidean / hungarian_assign / print_table）
        └── orchestrator.py                  # Orchestrator 类：A1→B1→A2→B2 串行驱动
```

### 启动链（`multi_robot_navigation.launch`）

```
Gazebo (orchard.world)
map_server (orchard_map.yaml)          ← 单实例，全局共享
  ↓
6 × <group ns="robotN">
  ├── robot_description (xacro robot_ns:=robotN)
  ├── spawn_model mecanum_robot_N (x/y 各不同)
  ├── robot_state_publisher (tf_prefix: robotN)
  ├── amcl（直接启动节点，注入 frame_id 参数）
  │     ├── remap static_map → /static_map   ← 必须，否则找不到地图
  │     ├── base_frame_id: robotN/base_footprint
  │     └── odom_frame_id: robotN/odom
  └── move_base
        ├── remap map → /map              ← 必须，否则全局 costmap 无静态层
        ├── global_costmap/robot_base_frame: robotN/base_footprint
        └── local_costmap/global_frame: robotN/odom
```

**单机器人 launch 的差异**：`orchard_navigation.launch` 通过 `turtlebot3_navigation/launch/amcl.launch` 启动 AMCL（frame_id 硬编码），不适用于多机器人场景。

TF 树（每台独立）：
```
map → robotN/odom → robotN/base_footprint → robotN/base_link → robotN/laser_link
```

### 任务分配（`task_assigner.py`）

从 `~robots` 参数加载机器人列表，为每台机器人启动独立线程：连接 `{namespace}/move_base`（30s 超时）→ 构造 `MoveBaseGoal`（`frame_id: map`）并发送 → `wait_for_result`（默认 180s）。所有线程并发，主线程 `join` 等待全部完成后打印结果并**退出**（非持久服务）。

目标配置在 `config/orchard_tasks.yaml`，修改后重启 `task_assignment.launch` 即生效，无需重编译。

### 六机器人布局（当前 `multi_robot_navigation.launch` spawn 坐标）

为配合 `orchard_orchestrator` 的 picker/transporter 角色划分，当前 6 台机器人的 spawn 坐标与 AMCL `initial_pose_*` 已全部改为如下布局。原布局（r1~r4 在 x=3.5、r5 北端、r6 东端）的备份在 `multi_robot_navigation.launch.bak`。

| ns     | 角色（orchestrator） | spawn 坐标    |
|--------|---------------------|---------------|
| robot1 | picker1             | (0.5, 3.0)    |
| robot2 | picker2             | (0.5, 5.0)    |
| robot3 | picker3             | (0.5, 7.0)    |
| robot4 | transporter1        | (0.75, 4.0)   |
| robot5 | transporter2        | (0.75, 2.0)   |
| robot6 | transporter3        | (0.75, 6.0)   |

修改 spawn 坐标时必须同步修改同一 `<group>` 内的 `amcl` 节点 `initial_pose_x/y/a`（每台 2 处、共 12 处），否则 AMCL 会以错误初值启动粒子滤波。

## 关键设计约束

**URDF 多机器人命名空间**：`mecanum_robot.urdf.xacro` 接受 `robot_ns` xacro 参数（默认为空，单机器人向后兼容）。设置后，激光和驱动插件的 topic/frame 均带前缀：
- `robotNamespace: robotN` → topic 变为 `/robotN/scan`、`/robotN/cmd_vel`
- `odometryFrame: robotN/odom`、`robotBaseFrame: robotN/base_footprint`

**多机器人 remap 规则**：在命名空间 `<group ns="robotN">` 内，服务和话题名均被相对解析。必须显式 remap：
- AMCL：`static_map → /static_map`（否则调用 `/robotN/static_map`，服务不存在）
- move_base：`map → /map`（否则全局 costmap 订阅 `/robotN/map`，找不到地图）

**URDF 驱动插件**：使用 `libgazebo_ros_planar_move`（非差速驱动）。接受 `cmd_vel`（linear.x/y + angular.z），直接覆盖物理引擎运动，发布 odom 并广播 `odom → base_footprint` TF。轮子关节仅作可视化。

**AMCL 里程计模型**：`amcl_params.yaml` 中 `odom_model_type: omni`（全向），不可改为 `diff`，否则麦克纳姆轮的横向运动会导致定位漂移。

**全向导航关键参数**：`dwa_local_planner_params_mecanum.yaml` 中 `holonomic_robot: true` 必须保留，否则 DWA 只生成 vx + ω，侧移能力失效。`y_vels: [-0.3, -0.1, 0.1, 0.3]` 控制侧向速度采样集合。

**廊道通行约束**：侧边通道有效宽度约 1.35m。`inflation_radius: 0.25`（`robot_radius: 0.20`）是在该宽度下保证路径可规划的临界值，不可随意增大。

**地图与场景对齐**：`orchard_map.pgm`（240×160px，0.05 m/px）由 Python+numpy 程序化生成，Y 轴翻转公式 `py = 159 - int(world_y / 0.05)`。如修改 `orchard.world` 的墙体位置，必须同步重新生成 PGM，否则 AMCL 定位会漂移。

## 坐标系

- 原点 (0,0) 在场景**左下角**，x→右（0–12m），y→上（0–8m）
- 四条廊道：Corridor 1（y=6~8）/ 2（y=4~6）/ 3（y=2~4）/ 4（y=0~2）
- 侧边通道：西侧 x=0~1.5，东侧 x=10.5~12

## 参数文件分工

move_base 加载两层参数，后加载者覆盖前者中同名 key：

| 文件 | 来源 | 作用 |
|------|------|------|
| `costmap_common_params_mecanum.yaml` | `mecanum_robot/param/` | robot_radius、inflation_radius、obstacle/raytrace range |
| `local/global_costmap_params.yaml` | `turtlebot3_navigation/param/` | costmap 层结构（静态层、障碍层等） |
| `move_base_params.yaml` | `turtlebot3_navigation/param/` | move_base 通用超时 / 恢复行为 |
| `dwa_local_planner_params_mecanum.yaml` | `mecanum_robot/param/` | 全向速度限制、holonomic_robot |
| `amcl_params.yaml` | `mecanum_robot/param/` | omni 里程计模型、粒子数等 |

多机器人 launch **不使用** `turtlebot3_navigation/launch/amcl.launch`（其 frame_id 硬编码，无法命名空间化），改为直接启动 `amcl` 节点并注入 frame_id 参数。

## 关联工作空间

`~/dogeman_ws`：兄弟工作空间，含 `turtlebot3_nav_bringup`（waffle_pi + turtlebot3_world）。可参考其 launch 文件模式，但两个工作空间不可混用 source。

## 匈牙利算法版任务分配器

```bash
# 终端 A：启动仿真（同主流程，无变化）
roslaunch orchard_navigation multi_robot_navigation.launch

# 终端 B：启动匈牙利算法版任务分配
roslaunch orchard_task_assignment task_assignment_hungarian.launch
```

`task_assigner_hungarian.py` 依赖 `scipy` 和 `numpy`，**不是 ROS 包**，无需修改 `package.xml`：

```bash
sudo apt-get install -y python3-scipy python3-numpy
```

配置文件 `orchard_tasks_hungarian.yaml` 的 `robots` 列表新增 `start` 字段（机器人出发坐标），顶层 `tasks` 列表单独列目标点（顺序故意打乱），算法自动求最优分配。详细说明见 `orchard_task_assignment/docs/hungarian_usage.md`。

## 多阶段编排器（`orchard_orchestrator`）

与 `orchard_task_assignment` 的**一次性**派发不同，`orchard_orchestrator` 实现 4 阶段串行的采摘-运输工作流。每个阶段独立调用匈牙利算法做机器人↔任务最优分配，阶段之间通过共享 `self.ready_boxes` 传递"果筐就绪"事件。

```
A1  3 picker 匈牙利分配 row_north 任务（西→东过走廊）   → 东端放 3 个果筐
B1  3 transporter 匈牙利分配 3 个果筐                  → 送回 depot
A2  3 picker 匈牙利分配 row_south 任务（东→西过走廊）   → 西端放 3 个果筐
B2  3 transporter 匈牙利分配第二批果筐                  → 送回 depot
```

**核心设计约束**：
- `config/orchard_layout.yaml` 的 `mode.pipeline: false` 为**唯一**支持的模式。`pipeline: true` 分支在 `orchestrator.py:run()` 中抛 `NotImplementedError`，不要在没有扩展实现的前提下切换
- picker A2 任务 yaw 必须为 `3.14159`（朝西），使机器人掉头后反向穿越走廊，避免无谓的走廊长途跨越
- `orchestrator` 通过订阅 `/robotN/amcl_pose`（`PoseWithCovarianceStamped`）实时获取机器人坐标用于代价矩阵。启动后 `run()` 先 `rospy.sleep(3.0)` 等待首帧 pose，否则首阶段 fallback 坐标会污染分配结果
- `picker_robots` / `transporter_robots` 的 namespace 列表必须与 `multi_robot_navigation.launch` 实际 spawn 出来的 6 台机器人**完全对应**，且 spawn 坐标应与 picker 在走廊西入口、transporter 聚集在 depot 附近这一物理布局一致
- 代价函数目前是**欧氏直线距离**（`hungarian_utils.euclidean`），不是 move_base 真实路径长度。这是有意为之的简化，升级为 `make_plan` 服务是已知改进方向
- 果筐 (`ready_boxes`) 是**纯数据**标记，不在 Gazebo 中建模实体物体，transporter 的"装载"只是 `rospy.sleep`

### 已修过的坑（改动前务必理解）

下面两个机制看起来冗余，但都是"三机器人同时朝同一格规划 → move_base 全员 state=4"的解药。删掉或简化它们会让 B 阶段重新崩盘。

- **`picking_tasks[*].stash`（picker 让位点）**：picker 到达 exit 后 `rospy.sleep(pick_dwell)` 模拟采摘，然后必须先移动到 stash 点**再**向 `ready_boxes` 登记果筐。否则 transporter 进来 pickup 时 picker 本体占据终点 cell，被 costmap 当作动态障碍，pickup 规划失败（B 阶段全员 state=4 的根因之一）。stash 不能落在同行走廊内（会挡后续 A2 反向穿行）：A1 用东侧通道 x=11.5 的纵向偏移，A2 用走廊中线 x=2.0 的纵向偏移。
- **`depot_slots`（transporter 独立卸货点）**：3 辆 transporter 必须各自有**不同的** depot 坐标，否则先到者停在 (0.75, 4.0) 会把后到两辆堵在 costmap 外（B 阶段返 depot 时全员 state=4 的另一根因）。当前配置让 robot4/5/6 分别返自己的 spawn cell (0.75, 4/2/6)。顶层 `depot` 字段仅作代价矩阵 fallback，不能用它替代 slots；新增 transporter 时必须同步补 slot。
- **根因签名**：若日志中同一阶段多个 transporter 同时 `state=4` 且目标点相同或相邻，几乎必然是"多机抢同一 goal cell"，而非导航栈本身问题。先查 goal 坐标是否唯一，再怀疑 costmap/inflation。

**单元自检**（无须 rosmaster）：
```bash
python3 src/orchard_orchestrator/scripts/hungarian_utils.py
```
预期：A1 对角分配 total=0；A2 `robot3→row1_south` 承担 6m 长跨度，total=6。若此自检失败，说明算法或坐标约定已偏离设计，先修它再动 orchestrator。

**依赖**：同匈牙利版任务分配器，需系统安装 `python3-scipy python3-numpy`（不进 `package.xml`）。
