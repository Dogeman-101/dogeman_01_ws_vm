# orchard_map

**VM 侧总启动 + 场景地图 + rviz 配置** 的配置包。

## 职责

这个包**不包含任何算法或节点代码**，是一个纯配置包：

1. VM 一键启动入口 `launch/vm_master.launch`
2. 场地地图（pgm + yaml）
3. rviz 可视化配置

## 文件组织

```
orchard_map/
├── launch/
│   ├── vm_master.launch         VM 总启动(动捕驱动 + 地图 + rviz)
│   └── vm_map_and_rviz.launch   地图 + rviz 的参数化轻量 launch
├── maps/
│   ├── animation_field.pgm      8m × 8m 空白占位地图
│   ├── animation_field.yaml
│   └── README.md                地图来源与参数说明
└── rviz/
    └── mvp.rviz                 MVP 阶段的 rviz 配置
```

## 用法

### 启动 VM 侧所有服务

```bash
source ~/catkin_mocap/devel/setup.bash     # $(find mocap_nokov) 需要它
source ~/dogeman_01_ws/devel/setup.bash
roslaunch orchard_map vm_master.launch
```

一条命令起动捕驱动 + 地图服务器 + rviz。

### 只起地图 + rviz（开发/调试用）

```bash
roslaunch orchard_map vm_map_and_rviz.launch
# 或切换地图:
roslaunch orchard_map vm_map_and_rviz.launch map_name:=other_field
# headless 环境不起 rviz:
roslaunch orchard_map vm_map_and_rviz.launch use_rviz:=false
```

### 换地图

新地图放到 `maps/`，然后传 `map_name:=新名字`（不含 `.yaml` 后缀）。无需改代码。

## 与相邻包的关系

- [mocap_localization](../mocap_localization/) — 动捕 → TF 的节点实现（`mocap_to_tf.py`）。本包**不依赖它**。
- `mocap_nokov`（`~/catkin_mocap/src/mocap_nokov`）— 动捕 UDP 驱动。本包通过 `$(find mocap_nokov)` 引用其 launch。
- 未来可能新增的 `orchard_orchestrator_real` — 任务编排器。届时会被 include 进 `vm_master.launch`。

## 地图约定

所有地图应满足：
- **原点对齐动捕原点**：yaml 的 `origin` 设为地图左下角在动捕坐标的位置，让地图中心落在 (0, 0)
- **分辨率 0.05 m/pixel**（标准值）
- **自由空间 = 255（白），障碍 = 0（黑）**

## 为什么把 `vm_map_and_rviz.launch` 从 mocap_localization 搬过来

原先它在 `mocap_localization/launch/`，但它加载的是 `mocap_localization/maps|rviz` 下的**配置**，并不调用 mocap_localization 的节点代码。职责归类应该跟着 `maps/` 和 `rviz/` 走——它们现在都在 `orchard_map`，launch 也跟过来，依赖关系就干净了（`$(find mocap_localization)` 消失）。
