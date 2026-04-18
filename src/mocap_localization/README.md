# mocap_localization

Subscribes to motion capture PoseStamped and broadcasts `map → odom` TF, replacing AMCL.

## Node: mocap_to_tf

### Subscriptions

| Topic | Type | Description |
|-------|------|-------------|
| `~mocap_pose_topic` (param) | geometry_msgs/PoseStamped | Robot pose in map frame from mocap |

### Published TF

| Parent | Child | Description |
|--------|-------|-------------|
| map | {robot_namespace}/odom | Correction transform (same role as AMCL) |

### Parameters

| Name | Default | Description |
|------|---------|-------------|
| `~mocap_pose_topic` | `/mocap_node/Robot_1/pose` | Mocap pose topic |
| `~robot_namespace` | `robot1` | Robot namespace for TF frame prefix |
| `~map_frame` | `map` | Global frame name |
| `~publish_rate` | `50.0` | TF broadcast rate (Hz) |

### Example

```bash
rosrun mocap_localization mocap_to_tf.py \
  _robot_namespace:=robot1 \
  _mocap_pose_topic:=/mocap_node/Robot_1/pose
```

### TODO

- Abstract into LocalizationProvider base class to support AMCL / Mocap / GT switching

## 仿真测试（Gazebo P3D 假动捕）

### 原理

Gazebo `libgazebo_ros_p3d` 插件以 100 Hz 发布每台车的 ground truth 位姿（`nav_msgs/Odometry`）。
`odom_to_pose` relay 节点将其转为 `geometry_msgs/PoseStamped`，话题名映射到 `/mocap_node/Robot_N/pose`，
使 `mocap_to_tf` 无需任何修改即可在仿真中运行。

### 话题流

| 阶段 | 话题 | 类型 |
|------|------|------|
| P3D 发布 | `/robot1/ground_truth_pose` | nav_msgs/Odometry |
| relay 转发 | `/mocap_node/Robot_1/pose` | geometry_msgs/PoseStamped |
| mocap_to_tf 订阅 | `/mocap_node/Robot_1/pose` | geometry_msgs/PoseStamped |

### 启动

```bash
# 为单台车启动 relay（仿真已跑起来后）
roslaunch mocap_localization gazebo_fake_mocap.launch \
  robot_namespace:=robot1 rigid_body_name:=Robot_1
```

### 单独测试 odom_to_pose

```bash
rosrun mocap_localization odom_to_pose.py \
  _odom_in:=/test_odom _pose_out:=/test_pose
```

## 目录变更说明（2026-04-18）

从本次重构开始：

- `maps/` 已迁移到 [`orchard_map/maps/`](../orchard_map/maps/)
- `rviz/` 已迁移到 [`orchard_map/rviz/`](../orchard_map/rviz/)
- `launch/vm_map_and_rviz.launch` 已迁移到 [`orchard_map/launch/vm_map_and_rviz.launch`](../orchard_map/launch/vm_map_and_rviz.launch)
- VM 侧的总启动 launch 在 [`orchard_map/launch/vm_master.launch`](../orchard_map/launch/vm_master.launch)

本包现在**专注于 `mocap_to_tf` / `odom_to_pose` 两个节点的 VM 版实现**，作为：
- 仿真环境下的动捕数据转 TF（配合 Gazebo + `gazebo_fake_mocap.launch`）
- 实机 `mocap_to_tf` 的"主版本"（W4A 的 `w4a_navigation` 会从这份源码保持同步）

注意：W4A 上 **`~/orchard_w4a_ws/src/w4a_navigation/scripts/mocap_to_tf.py`** 是实机部署的"生产版本"，修改本包代码时要手工同步 W4A 那份。
