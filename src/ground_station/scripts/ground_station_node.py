#!/usr/bin/env python3
"""ROS node that owns mission state and exposes normalized data to the GUI."""

import os
import sys
import json
import math
import threading
import time
from pathlib import Path

import rospy
from std_msgs.msg import String

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))

from ground_station.config import load_config  # noqa: E402
from ground_station.models import GridMission  # noqa: E402
from ground_station.ros_adapter import RosAdapter  # noqa: E402


class SitlFlightController:
    ALLOWED_MODES = {"POSCTL", "ALTCTL", "AUTO.LOITER", "AUTO.LAND", "AUTO.RTL", "AUTO.TAKEOFF", "OFFBOARD"}

    def __init__(self, config):
        from geometry_msgs.msg import PoseStamped
        from mavros_msgs.msg import State
        from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

        settings = config["mavros_control"]
        topics = config.get("ros", {}).get("topics", {})
        queue_size = int(config.get("ros", {}).get("queue_size", 10))
        self.state = None
        self.pose_type = PoseStamped
        self.setpoint_rate_hz = float(settings.get("setpoint_rate_hz", 20.0))
        self.prestream_count = int(settings.get("offboard_prestream_count", 100))
        self.target_lock = threading.Lock()
        self.target_altitude = float(settings.get("default_takeoff_altitude_m", 2.5))
        self.setpoint_active = threading.Event()
        self.minimum_altitude = float(settings.get("min_altitude_m", 0.5))
        self.maximum_altitude = float(settings.get("max_altitude_m", 20.0))
        self.state_timeout = float(settings.get("state_timeout_s", 5.0))
        self.service_timeout = float(settings.get("service_timeout_s", 10.0))
        self.arm_service = rospy.ServiceProxy(settings.get("arming_service", "/mavros/cmd/arming"), CommandBool)
        self.mode_service = rospy.ServiceProxy(settings.get("set_mode_service", "/mavros/set_mode"), SetMode)
        self.takeoff_service = rospy.ServiceProxy(settings.get("takeoff_service", "/mavros/cmd/takeoff"), CommandTOL)
        self.setpoint_publisher = rospy.Publisher(settings.get("setpoint_topic", "/mavros/setpoint_position/local"), PoseStamped, queue_size=10)
        self.state_subscriber = rospy.Subscriber(settings.get("state_topic", "/mavros/state"), State, self._state_callback, queue_size=queue_size)
        self.command_subscriber = rospy.Subscriber(topics.get("command", "/uav/ground_station_command"), String, self._command_callback, queue_size=queue_size)
        self.setpoint_thread = threading.Thread(target=self._publish_setpoints, daemon=True)
        self.setpoint_thread.start()

    def _state_callback(self, state):
        self.state = state

    def _require_connection(self):
        if self.state is None or not self.state.connected:
            raise RuntimeError("MAVROS 尚未连接到飞控")

    def _wait_for_state(self, predicate, description):
        deadline = time.monotonic() + self.state_timeout
        while time.monotonic() < deadline and not rospy.is_shutdown():
            state = self.state
            if state is not None and predicate(state):
                return state
            time.sleep(0.05)
        raise RuntimeError("等待飞控确认{}超时".format(description))

    def _call_service(self, service, *args, **kwargs):
        service_name = getattr(service, "resolved_name", None) or getattr(service, "service_name", None)
        if service_name:
            rospy.wait_for_service(service_name, timeout=self.service_timeout)
        return service(*args, **kwargs)

    def _set_mode(self, mode):
        if mode == "OFFBOARD" and not self.setpoint_active.is_set():
            self._start_offboard_setpoints()
        response = self._call_service(self.mode_service, base_mode=0, custom_mode=mode)
        if not response.mode_sent:
            raise RuntimeError("飞控未接受模式切换: {}".format(mode))
        self._wait_for_state(lambda state: state.mode == mode, "切换到 {} 模式".format(mode))

    def _publish_setpoints(self):
        rate = rospy.Rate(self.setpoint_rate_hz)
        while not rospy.is_shutdown():
            if self.setpoint_active.is_set():
                with self.target_lock:
                    altitude = self.target_altitude
                pose = self.pose_type()
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = 0.0
                pose.pose.position.y = 0.0
                pose.pose.position.z = altitude
                pose.pose.orientation.w = 1.0
                self.setpoint_publisher.publish(pose)
            rate.sleep()

    def _start_offboard_setpoints(self, altitude=None):
        if altitude is not None:
            with self.target_lock:
                self.target_altitude = float(altitude)
        self.setpoint_active.set()
        deadline = time.monotonic() + 2.0
        while self.setpoint_publisher.get_num_connections() == 0 and time.monotonic() < deadline:
            if rospy.is_shutdown():
                raise RuntimeError("ROS 正在关闭，无法启动 Offboard setpoint")
            time.sleep(0.05)
        if self.setpoint_publisher.get_num_connections() == 0:
            raise RuntimeError("MAVROS 未订阅 setpoint 话题")
        rate = rospy.Rate(self.setpoint_rate_hz)
        for _ in range(self.prestream_count):
            if rospy.is_shutdown():
                raise RuntimeError("ROS 正在关闭，无法完成 Offboard 预发送")
            with self.target_lock:
                target_altitude = self.target_altitude
            pose = self.pose_type()
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = 0.0
            pose.pose.position.y = 0.0
            pose.pose.position.z = target_altitude
            pose.pose.orientation.w = 1.0
            self.setpoint_publisher.publish(pose)
            rate.sleep()

    def _ensure_armed(self):
        if self.state is not None and self.state.armed:
            return
        response = self._call_service(self.arm_service, True)
        if not response.success:
            raise RuntimeError("飞控拒绝起飞前解锁，请检查 QGC 中的预解锁检查信息")
        self._wait_for_state(lambda state: state.armed, "解锁")
        rospy.loginfo("SITL: vehicle armed")

    def _command_callback(self, message):
        try:
            request = json.loads(message.data)
            command = request.get("command")
            payload = request.get("payload") or {}
            if not isinstance(payload, dict):
                raise ValueError("命令 payload 格式无效")
            if command in {"arm", "disarm", "takeoff", "set_mode"}:
                self._require_connection()
            if command == "arm":
                response = self._call_service(self.arm_service, True)
                if not response.success:
                    raise RuntimeError("飞控拒绝解锁")
                self._wait_for_state(lambda state: state.armed, "解锁")
                rospy.loginfo("SITL: vehicle armed")
            elif command == "disarm":
                response = self._call_service(self.arm_service, False)
                if not response.success:
                    raise RuntimeError("飞控拒绝上锁")
                self._wait_for_state(lambda state: not state.armed, "上锁")
                rospy.loginfo("SITL: vehicle disarmed")
            elif command == "set_mode":
                mode = payload.get("mode")
                if mode not in self.ALLOWED_MODES:
                    raise ValueError("不支持的 PX4 模式: {}".format(mode))
                self._set_mode(mode)
                rospy.loginfo("SITL: requested mode %s", mode)
            elif command == "takeoff":
                altitude = payload.get("altitude_m")
                if isinstance(altitude, bool) or not isinstance(altitude, (int, float)) or not math.isfinite(altitude):
                    raise ValueError("起飞高度必须是有效数字")
                if not self.minimum_altitude <= altitude <= self.maximum_altitude:
                    raise ValueError("起飞高度范围为 {:.1f} 至 {:.1f} 米".format(self.minimum_altitude, self.maximum_altitude))
                self._start_offboard_setpoints(altitude)
                self._set_mode("OFFBOARD")
                self._ensure_armed()
                rospy.loginfo("SITL: Offboard takeoff target set to %.1f m", altitude)
        except (ValueError, RuntimeError, TypeError, json.JSONDecodeError) as error:
            rospy.logwarn("SITL flight command rejected: %s", error)
        except rospy.ServiceException as error:
            rospy.logerr("MAVROS service call failed: %s", error)


class GroundStationNode:
    def __init__(self, config):
        self.config = config
        self.mission = GridMission(config["grid"]["rows"], config["grid"]["columns"])
        self.adapter = RosAdapter(config)
        self.flight_controller = None
        control = config.get("mavros_control", {})
        if config.get("profile") == "sitl" and control.get("enabled", False):
            self.flight_controller = SitlFlightController(config)
        else:
            rospy.loginfo("Direct MAVROS flight controls are disabled for profile '%s'", config.get("profile", "unknown"))

    def run(self):
        rospy.loginfo("Ground station node started with profile '%s'", self.config.get("profile", "unknown"))
        rospy.spin()


def main():
    rospy.init_node("ground_station_node")
    config_file = rospy.get_param("~config_file", "")
    if not config_file:
        raise RuntimeError("~config_file is required")
    GroundStationNode(load_config(os.path.expanduser(config_file))).run()


if __name__ == "__main__":
    main()