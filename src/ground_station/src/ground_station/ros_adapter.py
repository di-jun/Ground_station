"""ROS transport adapter. Business code only sees normalized callbacks."""

import json

from std_msgs.msg import String

try:
    import rospy
except ImportError:  # Allows importing the model in non-ROS unit tests.
    rospy = None


class RosAdapter:
    def __init__(self, config: dict):
        if rospy is None:
            raise RuntimeError("ROS is not available; source /opt/ros/noetic/setup.bash first")
        topics = config.get("ros", {}).get("topics", {})
        queue_size = int(config.get("ros", {}).get("queue_size", 10))
        self.command_publisher = rospy.Publisher(topics.get("command", "/uav/ground_station_command"), String, queue_size=queue_size)

    def publish_command(self, command: str, **payload) -> None:
        message = {"command": command, "payload": payload}
        self.command_publisher.publish(String(data=json.dumps(message, sort_keys=True)))