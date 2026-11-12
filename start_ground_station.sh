#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/noetic/setup.bash
cd /home/cai/Ground_station
source devel/setup.bash

echo "[ground_station] Starting ROS environment..."

if rostopic list 2>/dev/null | grep -q '^/mavros/local_position/odom$'; then
  echo "[ground_station] Bridging /mavros/local_position/odom -> /uav/local_position/odom"
  rosrun topic_tools relay /mavros/local_position/odom /uav/local_position/odom &
else
  echo "[ground_station] Warning: /mavros/local_position/odom not found. Start PX4 SITL + MAVROS first."
fi

if rostopic list 2>/dev/null | grep -q '^/mavros/state$'; then
  echo "[ground_station] Bridging /mavros/state -> /uav/state"
  python3 - <<'PY' &
import rospy
from std_msgs.msg import String
from mavros_msgs.msg import State

rospy.init_node('ground_station_state_bridge', anonymous=True)
pub = rospy.Publisher('/uav/state', String, queue_size=10)

def cb(msg: State):
    text = 'connected' if msg.connected else 'disconnected'
    pub.publish(String(data=text))

rospy.Subscriber('/mavros/state', State, cb, queue_size=10)
rospy.spin()
PY
else
  echo "[ground_station] Warning: /mavros/state not found. Start PX4 SITL + MAVROS first."
fi

echo "[ground_station] Starting ground_station node..."
roslaunch ground_station ground_station.launch &

echo "[ground_station] Starting GUI..."
roslaunch ground_station ground_station_gui.launch
