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
