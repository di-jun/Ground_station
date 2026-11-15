# PX4 地面站

ROS Noetic/catkin 地面站，当前默认使用 PX4 SITL、MAVROS 和 `sitl.yaml` 配置。

## 功能

- 提供 Qt 网格 GUI。默认网格为 8 行、7 列，行标从上到下为 H 到 A，列标从左到右为 1 到 7。
- 支持编辑和清空禁飞格。输入 `A1,+B2,-C3,D5` 可增加 A1、B2、D5 并移除 C3；不带前缀的格子也表示增加。更新禁飞格后需要重新生成航线。
- 使用 `M` 或“生成航线”按钮生成覆盖航线。航线会避开禁飞格，并尝试保持相邻航点连续；禁飞区隔断可巡查区域时会提示无法生成。
- “发送航线任务”将已规划航点发布到 ROS 命令话题并装载到节点，不会单独触发起飞。
- SITL 配置下提供 PX4 模式切换、解锁、上锁和起飞控制。起飞高度可设为 `1.5–20m`。
- Offboard 控制以 20Hz 发布本地位置 setpoint，切换 `OFFBOARD` 前预发送 100 条，并等待飞控确认模式和解锁状态。
- 航点到达判定使用配置的 `waypoint_tolerance_m`；默认到达任务最后一个航点后请求 `AUTO.LAND`，可通过配置关闭。
- `hardware.yaml` 不启用直接 MAVROS 飞行控制。当前 MAVROS 飞行控制只在 SITL profile 下创建，不应据此认为硬件飞行链路已验证。

## 启动

启动地面站前，必须先启动 PX4 SITL 和 MAVROS。地面站启动脚本不会启动 PX4 或 MAVROS。以下命令假定 ROS Noetic、catkin 工作区及 PX4 SITL 环境均已安装并配置。

使用本项目的网格世界启动 PX4 SITL：

```bash
roslaunch px4 mavros_posix_sitl.launch
```

该 launch 默认将无人机生成在 H7 单元中心 `(x=6, y=0, z=0)`，机头朝向 `+y`，即朝向顶部的数字 7。需要临时覆盖位置或朝向时，可在命令末尾传入 `x:=`、`y:=`、`z:=` 或 `Y:=` 参数。

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
- 将 `/mavros/local_position/odom` 中继到 `/uav/local_position/odom`，并把 `/mavros/state` 的连接状态转换为 `/uav/state` 字符串消息。
- 启动 `ground_station_node`。
- 启动 Qt GUI。

启动脚本内工作区路径固定为 `/home/cai/Ground_station`。若工作区放在其他路径，需要先调整脚本中的路径。

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

## 操作流程

1. 在禁飞区输入框中填写要增加或移除的网格，点击“应用禁飞区”；也可点击“清空禁飞区”。
2. 点击“生成航线”或按 `M`，检查网格上的航线和航点数。
3. 点击“发送航线任务”将航点装载到控制节点。任务航点高度取当前起飞高度设置值（飞行控制关闭时取网格配置高度）。
4. SITL 下设置起飞高度并点击“起飞”。节点检查 MAVROS 连接和 setpoint 订阅者，以 20Hz 预发送 100 条目标，再切换 `OFFBOARD` 并解锁。若已装载航线，起飞目标为首个航点；否则目标为原点上方的设定高度。
5. 起飞后节点持续发布当前航点的位置目标，并在无人机进入容差范围后推进到下一航点。到达最后一个航点后默认请求 PX4 `AUTO.LAND`。

起飞前必须保证 MAVROS 已订阅 `/mavros/setpoint_position/local`。如果该话题显示 `Publishers: None`，地面站节点尚未创建发布器；如果显示 `Subscribers: None`，MAVROS 未运行或相关插件未加载。解锁或模式切换被拒绝时，查看启动日志及 QGroundControl 中的 PX4 预解锁检查信息。

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

任务命令使用 `std_msgs/String` JSON 消息，默认话题为 `/uav/ground_station_command`。`prepare_mission` 命令携带 `route` 航点数组，节点校验坐标后将其设为当前任务；它本身不切换模式或解锁。位置和状态话题分别为 `/uav/local_position/odom`、`/uav/state`；启动脚本负责从 MAVROS 转发这些数据。

地图坐标以 `H7=(0, 0)` 为原点，机头方向为 `+y`，机头左侧为 `+x`；因此 `H6=(1, 0)`、`G7=(0, 1)`、`H1=(6, 0)`、`A7=(0, 7)`。GUI 当前每格按 1 米转换并发送航点，虽然 YAML 中有 `grid.cell_size_m: 0.5`，该值目前没有用于 GUI 航点换算。地图坐标的 x 轴与 ROS/MAVROS 右手世界坐标系相反，节点发布航点时转换为 MAVROS `(x, y, z)=(-地图x, 地图y, 高度)`，到点判定使用相同坐标系转换。位置 setpoint 的 yaw 默认保持朝向 `+y`。

SITL 飞行控制配置在 `mavros_control` 下，包括 MAVROS 状态/服务/setpoint 接口、setpoint 频率、预发送数量、起飞高度范围、航点容差和任务结束自动降落选项。硬件 profile 不加载该飞行控制器；`pw_link` 当前配置为关闭，项目未提供可用的串口遥测控制实现。真实飞机或不同 ROS 命名空间不应只通过修改 YAML 直接启用；必须先验证消息类型、坐标系、单位和飞控安全检查。

## 测试

```bash
source /opt/ros/noetic/setup.bash
cd /home/cai/Ground_station
PYTHONPATH=src/ground_station/src python3 -m unittest discover -s src/ground_station/test
```

当前单元测试覆盖网格坐标转换、覆盖航线连续性、禁飞格处理和部分 SITL 控制逻辑；测试不替代 PX4 SITL 中的实际飞行与安全验证。