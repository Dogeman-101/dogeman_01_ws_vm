# orchard_orchestrator

多机器人果园采摘-运输任务编排器。在 `orchard_navigation` 的 6 机器人仿真之上，
编排 **3 个 picker + 3 个 transporter** 执行两轮"采摘→运输"工作流，每一阶段的
任务分配都用匈牙利算法（`scipy.optimize.linear_sum_assignment`）求解。

## 工作流

```
A1  3 picker 匈牙利分配 row_north 任务（西→东）   → 3 个果筐放在东端
B1  3 transporter 匈牙利分配 3 个果筐             → 送回 depot
A2  3 picker 匈牙利分配 row_south 任务（东→西）   → 3 个果筐放在西端
B2  3 transporter 匈牙利分配 3 个果筐             → 送回 depot
```

本版本只实现**串行模式** `pipeline: false`。
`pipeline: true` 流水线分支保留占位但抛 `NotImplementedError`。

## 系统依赖

除标准 ROS Noetic 外需手动安装：

```bash
sudo apt-get install -y python3-scipy python3-numpy
```

（与 `orchard_task_assignment` 的匈牙利版一致，不作为 ROS 包依赖声明）

## 使用

```bash
# 终端 A：启动 6 机器人仿真（会读改过的 spawn 坐标）
source ~/dogeman_01_ws/devel/setup.bash
roslaunch orchard_navigation multi_robot_navigation.launch
# 等 30s 观察 6 台机器人在 Gazebo/RViz 中定位收敛

# 终端 B：启动编排器
source ~/dogeman_01_ws/devel/setup.bash
roslaunch orchard_orchestrator orchestrator.launch
```

编排器打印每阶段的匈牙利分配表、每个机器人的 goal 日志，
4 阶段完成后打印总耗时汇总并退出。

## 文件布局

```
orchard_orchestrator/
├── config/orchard_layout.yaml     # depot + 6 任务 + picker/transporter 分组 + 时间参数
├── launch/orchestrator.launch     # 单节点 + rosparam load
├── scripts/
│   ├── hungarian_utils.py         # 纯函数：euclidean / hungarian_assign / print_table
│   └── orchestrator.py            # Orchestrator 类 + 4 阶段串行驱动
├── package.xml
├── CMakeLists.txt
└── README.md
```

## 机器人编组约定

本包**不修改** `orchard_navigation`、`mecanum_robot`、`orchard_task_assignment`，
但**修改了** `orchard_navigation/launch/multi_robot_navigation.launch` 的 6 台机器人
初始 spawn 坐标和 AMCL `initial_pose_*`（共 12 处），以匹配 picker/transporter 的
角色布局。原文件已备份为 `multi_robot_navigation.launch.bak`。

| ns     | 角色         | 初始位置      |
|--------|--------------|---------------|
| robot1 | picker1      | (0.5, 3.0)    |
| robot2 | picker2      | (0.5, 5.0)    |
| robot3 | picker3      | (0.5, 7.0)    |
| robot4 | transporter1 | (0.75, 4.0)   |
| robot5 | transporter2 | (0.75, 2.0)   |
| robot6 | transporter3 | (0.75, 6.0)   |

## 设计简化点

- 代价函数使用**欧氏直线距离**，不是 move_base 真实路径长度。后续可升级为调用
  `make_plan` 服务或预计算离线路径成本表
- 任务调度为硬编码 4 阶段，无动态插入/重分配
- 任务失败当前只记录 `False` 结果，不做重试或任务再分配
- "果筐"是纯数据标记（`self.ready_boxes[task_id] = (x, y)`），不在 Gazebo 中建模物体

## 单元自检

`hungarian_utils.py` 主函数内置手算样例，可无须 rosmaster 直接运行：

```bash
python3 src/orchard_orchestrator/scripts/hungarian_utils.py
```

预期输出：A1 对角分配（总 cost=0）、A2 非平凡分配（robot3 承担 6m 长跨度，总 cost=6）。
