# 匈牙利算法版任务分配器使用说明

## 前置依赖

```bash
# Ubuntu 20.04 用 apt 安装（推荐，无需 pip）
sudo apt-get install -y python3-scipy python3-numpy
```

## 用法

```bash
# 终端 A：启动 6 机器人仿真（与主流程相同，无需改动）
roslaunch orchard_navigation multi_robot_navigation.launch

# 终端 B：待 Gazebo 和导航栈稳定后，启动匈牙利算法版任务分配
roslaunch orchard_task_assignment task_assignment_hungarian.launch
```

节点启动后会先打印分配矩阵（哪台机器人去哪个任务点、各自代价），然后并发执行导航，全部完成后自动退出。

---

## 匈牙利算法在这里做了什么

**背景**：6 台机器人，6 个任务点，需要决定谁去哪里，使总行驶距离最短。

**代价矩阵**是一张 6×6 的表格，第 i 行第 j 列的数字是"机器人 i 从出发点走到任务 j 的直线距离"。这张表格由代码自动算出，不需要手动填写。

**匈牙利算法**（`scipy.optimize.linear_sum_assignment`）读入这张表，找到一种"每行选一列、每列只被选一次"的方案，使选出的数字加总最小——这就是总路程最短的分配方案。

**为什么用直线距离**：直线距离是最快的"粗估"方法，计算只需几微秒。后续如果精度要求更高，可以换成调用 `move_base` 的 `make_plan` 服务来获取真实规划路径的长度，但算法本身不需要任何改动，只需替换代价矩阵的计算方式。

**实测效果**：配置文件故意打乱了任务顺序，顺序分配的总代价约 37.2 m，匈牙利算法的最优解约 34.0 m，节省约 9%。可以在节点启动时的日志里看到交叉分配的结果（例如 robot1 被分配到 tasks[2] 而非 tasks[0]）。
