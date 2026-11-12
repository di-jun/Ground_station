# PX4 地面站

ROS Noetic/catkin 地面站，当前默认使用 PX4 SITL、MAVROS 和 `sitl.yaml` 配置。

## 功能

- Qt GUI 显示 9x7 网格、禁飞区和覆盖航线。
- 使用 `M` 或 GUI 按钮生成航线，并通过 ROS 命令话题发送任务准备命令。
- 通过 MAVROS 解锁、上锁和切换 PX4 飞行模式。
- 使用 PX4 Offboard 位置控制起飞：以 20Hz 发布本地位置 setpoint，预发送 100 条后切换 `OFFBOARD`，确认模式成功后解锁，并持续保持目标高度。
- 默认起飞高度范围为 `2.5–20m`。
- `hardware.yaml` 默认关闭直接 MAVROS 飞行控制；当前飞行控制仅用于 SITL 仿真。

## 启动

启动地面站前，必须先启动 PX4 SITL 和 MAVROS。地面站启动脚本不会启动 PX4 或 MAVROS。

确认 MAVROS 已连接：

```bash
rostopic echo -n 1 /mavros/state
```

输出中应包含：

```text
connected: True
```

确认 MAVROS 提供必要的控制接口：

```bash
rosservice list | grep -E '/mavros/(cmd/arming|set_mode)'
rostopic info /mavros/setpoint_position/local
```

确认 `/mavros/setpoint_position/local` 的订阅者包含 `/mavros` 后，在一个终端直接运行：

```bash
/home/cai/Ground_station/start_ground_station.sh
```

脚本会自动完成以下工作：

- 加载 `/opt/ros/noetic/setup.bash` 和工作区 `devel/setup.bash`。
- 将 MAVROS 的本地位置和状态话题桥接到地面站配置的话题。
- 启动 `ground_station_node`。
- 启动 Qt GUI。

首次构建或代码更新后，可先执行：

```bash
source /opt/ros/noetic/setup.bash
cd /home/cai/Ground_station
catkin_make
```

如果脚本没有执行权限，可以执行一次：

```bash
chmod +x /home/cai/Ground_station/start_ground_station.sh
```

## Offboard 起飞流程

在 GUI 中点击“起飞”后，节点按以下顺序执行：

1. 检查 MAVROS 与飞控的连接状态。
2. 将目标位置设为 `x=0`、`y=0`、`z=起飞高度`。
3. 以 20Hz 向 `/mavros/setpoint_position/local` 预发送 100 条位置目标。
4. 调用 `/mavros/set_mode` 切换到 `OFFBOARD`。
5. 等待 `/mavros/state` 确认模式为 `OFFBOARD`。
6. 调用 `/mavros/cmd/arming` 解锁。
7. 持续发布目标位置，保持无人机在目标高度。

因此，起飞前必须保证 PX4 已经订阅 `/mavros/setpoint_position/local`。如果该话题显示 `Publishers: None`，表示地面站节点尚未启动或没有成功创建发布器；如果显示 `Subscribers: None`，表示 MAVROS 没有运行或相关插件没有加载。

## 常见检查

```bash
rosnode list | grep -E 'mavros|ground_station'
rostopic info /mavros/setpoint_position/local
rostopic echo -n 1 /mavros/state
rosservice list | grep -E '/mavros/(cmd/arming|set_mode)'
```

起飞失败时，查看启动脚本输出的 `ground_station_node` 日志，并检查 QGroundControl 中的 PX4 预解锁检查提示。常见原因包括 MAVROS 未连接、Offboard setpoint 话题没有订阅者、PX4 未接受 `OFFBOARD` 模式或安全检查未通过。

## 话题配置

默认 SITL 配置位于：

```text
/home/cai/Ground_station/src/ground_station/config/sitl.yaml
```

主要 MAVROS 接口为：

```text
/mavros/state
/mavros/cmd/arming
/mavros/set_mode
/mavros/setpoint_position/local
```

任务命令使用 `std_msgs/String` JSON 消息，默认话题为 `/uav/ground_station_command`。真实飞机或不同 ROS 命名空间不应只通过修改 YAML 直接启用；必须先验证消息类型、坐标系、单位和飞控安全检查。

## 测试

```bash
source /opt/ros/noetic/setup.bash
cd /home/cai/Ground_station
PYTHONPATH=src/ground_station/src python3 -m unittest discover -s src/ground_station/test
```