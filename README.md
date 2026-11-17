# PX4 地面站

## 这是什么

这是一个基于 ROS Noetic 和 PyQt5 的 PX4 无人机地面站原型，提供网格地图界面，并通过 MAVROS 与 PX4 SITL 通信。

## 有什么用

- 在网格地图上设置禁飞区域并生成覆盖航线。
- 将航线发送给 ROS 节点，并在 PX4 SITL 中进行起飞、航点飞行和降落控制。

当前飞行控制仅面向 SITL；真实硬件飞行链路尚未验证。

## 环境安装

推荐使用 Ubuntu 20.04。先按照 [ROS Noetic 官方安装说明](https://wiki.ros.org/noetic/Installation/Ubuntu) 配置 ROS 软件源，然后安装 ROS、MAVROS 和界面依赖：

```bash
sudo apt update
sudo apt install -y ros-noetic-desktop-full ros-noetic-mavros ros-noetic-mavros-extras python3-yaml python3-pyqt5
sudo /opt/ros/noetic/lib/mavros/install_geographiclib_datasets.sh
```

安装 PX4 SITL：

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash PX4-Autopilot/Tools/setup/ubuntu.sh
```

运行 PX4 安装脚本后，按提示完成系统依赖安装并重新登录。更多 PX4 安装信息见 [PX4 官方开发环境说明](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html)。

## 如何使用

在项目目录中构建工作区：

```bash
source /opt/ros/noetic/setup.bash
cd /home/cai/Ground_station
catkin_make
chmod +x start_ground_station.sh
```

先在一个终端启动 PX4 SITL 和 MAVROS：

```bash
roslaunch px4 mavros_posix_sitl.launch
```

再在另一个终端启动地面站：

```bash
./start_ground_station.sh
```

在界面中设置禁飞格，生成航线并发送任务；SITL 下可继续设置起飞高度并执行起飞。启动脚本默认工作区路径为 `/home/cai/Ground_station`，若项目位于其他位置，请先修改脚本中的路径。