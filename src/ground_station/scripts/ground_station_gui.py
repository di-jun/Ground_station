#!/usr/bin/env python3
"""Qt ground station GUI for keep-out editing and route planning."""

import json
import math
import os
import sys
import time
from pathlib import Path

from PyQt5.QtCore import QPointF, QRectF, Qt
from PyQt5.QtGui import QColor, QPainter, QPen
from PyQt5.QtWidgets import QApplication, QComboBox, QDoubleSpinBox, QHBoxLayout, QLabel, QLineEdit, QMainWindow, QMessageBox, QPushButton, QVBoxLayout, QWidget

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "src"))

from ground_station.config import load_config  # noqa: E402
from ground_station.models import GridMission  # noqa: E402

try:
    import rospy
    from std_msgs.msg import String
except ImportError:
    rospy = None


class GridWidget(QWidget):
    def __init__(self, mission):
        super().__init__()
        self.mission = mission
        self.setMinimumSize(420, 360)

    def paintEvent(self, event):
        del event
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        left_margin = 32
        top_margin = 28
        right_margin = 12
        bottom_margin = 12
        grid_width = self.width() - left_margin - right_margin
        grid_height = self.height() - top_margin - bottom_margin
        cell_w = grid_width / self.mission.columns
        cell_h = grid_height / self.mission.rows
        route_cells = set(self.mission.route)

        # Keep the coordinate labels outside the cells so the board reads as a
        # normal 2D grid: columns 1-7 across the top and rows A-H down the side.
        painter.setPen(QPen(QColor("#30383d"), 1))
        for column in range(self.mission.columns):
            x = left_margin + column * cell_w
            painter.drawText(
                QRectF(x, 0, cell_w, top_margin),
                Qt.AlignCenter,
                str(column + 1),
            )
        for row in range(self.mission.rows):
            y = top_margin + row * cell_h
            painter.drawText(
                QRectF(0, y, left_margin, cell_h),
                Qt.AlignCenter,
                chr(ord("A") + row),
            )

        painter.setPen(QPen(QColor("#aab2b8"), 1))
        for row in range(self.mission.rows):
            for col in range(self.mission.columns):
                rect_x = left_margin + col * cell_w
                rect_y = top_margin + row * cell_h
                cell = (row, col)
                color = QColor("#dce3e7")
                if cell in route_cells:
                    color = QColor("#d9ead3")
                if cell in self.mission.no_fly_cells:
                    color = QColor("#efb4aa")
                painter.fillRect(int(rect_x), int(rect_y), int(cell_w), int(cell_h), color)
                painter.drawRect(int(rect_x), int(rect_y), int(cell_w), int(cell_h))
        painter.setPen(QPen(QColor("#24734b"), 2))
        for first, second in zip(self.mission.route, self.mission.route[1:]):
            x1 = left_margin + (first[1] + 0.5) * cell_w
            y1 = top_margin + (first[0] + 0.5) * cell_h
            x2 = left_margin + (second[1] + 0.5) * cell_w
            y2 = top_margin + (second[0] + 0.5) * cell_h
            painter.drawLine(int(x1), int(y1), int(x2), int(y2))
            self._draw_route_arrow(painter, x1, y1, x2, y2, cell_w, cell_h)

    @staticmethod
    def _draw_route_arrow(painter, x1, y1, x2, y2, cell_w, cell_h):
        """Draw an arrow for one traversal, including repeated return segments."""
        dx = x2 - x1
        dy = y2 - y1
        length = math.hypot(dx, dy)
        if length == 0:
            return

        # Place the head away from the exact midpoint so opposite traversals
        # on the same edge remain visible as two different directions.
        head_x = x1 + dx * 0.62
        head_y = y1 + dy * 0.62
        arrow_size = min(cell_w, cell_h) * 0.14
        unit_x = dx / length
        unit_y = dy / length
        wing_x = -unit_x * arrow_size
        wing_y = -unit_y * arrow_size
        side_x = -unit_y * arrow_size * 0.55
        side_y = unit_x * arrow_size * 0.55
        painter.drawLine(
            QPointF(head_x, head_y),
            QPointF(head_x + wing_x + side_x, head_y + wing_y + side_y),
        )
        painter.drawLine(
            QPointF(head_x, head_y),
            QPointF(head_x + wing_x - side_x, head_y + wing_y - side_y),
        )


class GroundStationWindow(QMainWindow):
    def __init__(self, config):
        super().__init__()
        self.config = config
        self.mission = GridMission(config["grid"]["rows"], config["grid"]["columns"])
        self.command_publisher = None
        flight_control = config.get("mavros_control", {})
        self.flight_controls_enabled = config.get("profile") == "sitl" and flight_control.get("enabled", False)
        self.setWindowTitle("PX4 Ground Station | {}".format(config.get("profile", "unknown").upper()))
        self.resize(1120, 720)

        root = QWidget()
        layout = QVBoxLayout(root)
        header = QHBoxLayout()
        self.status_label = QLabel("Transport: ROS | Profile: {} | Link: disconnected".format(config.get("profile", "unknown")))
        header.addWidget(self.status_label, 1)
        self.no_fly_edit = QLineEdit()
        self.no_fly_edit.setPlaceholderText("例：A1,+B2,-C3,D5 ；+表示增加，-表示删除")
        self.no_fly_edit.setMaximumWidth(420)
        header.addWidget(self.no_fly_edit)
        set_button = QPushButton("应用禁飞区")
        set_button.clicked.connect(self.apply_no_fly_cells)
        header.addWidget(set_button)
        clear_button = QPushButton("清空禁飞区")
        clear_button.clicked.connect(self.clear_no_fly_cells)
        header.addWidget(clear_button)
        plan_button = QPushButton("生成航线 (M)")
        plan_button.clicked.connect(self.plan_route)
        header.addWidget(plan_button)
        layout.addLayout(header)

        map_layout = QHBoxLayout()
        self.grid_widget = GridWidget(self.mission)
        map_layout.addWidget(self.grid_widget, 3)
        side = QVBoxLayout()
        side.addWidget(QLabel("任务控制"))
        self.task_status = QLabel("未规划")
        side.addWidget(self.task_status)
        self.route_info = QLabel("航点：0")
        side.addWidget(self.route_info)
        start_button = QPushButton("发送航线任务")
        start_button.clicked.connect(lambda: self.send_command("prepare_mission"))
        side.addWidget(start_button)
        if self.flight_controls_enabled:
            side.addWidget(QLabel("SITL 飞行控制"))
            self.mode_selector = QComboBox()
            self.mode_selector.addItem("位置控制 (POSCTL)", "POSCTL")
            self.mode_selector.addItem("高度控制 (ALTCTL)", "ALTCTL")
            self.mode_selector.addItem("悬停 (AUTO.LOITER)", "AUTO.LOITER")
            self.mode_selector.addItem("返航 (AUTO.RTL)", "AUTO.RTL")
            self.mode_selector.addItem("降落 (AUTO.LAND)", "AUTO.LAND")
            self.mode_selector.addItem("位置 Offboard (OFFBOARD)", "OFFBOARD")
            side.addWidget(self.mode_selector)
            mode_button = QPushButton("切换飞行模式")
            mode_button.clicked.connect(self.set_flight_mode)
            side.addWidget(mode_button)
            arm_layout = QHBoxLayout()
            arm_button = QPushButton("解锁")
            arm_button.clicked.connect(self.arm_vehicle)
            arm_layout.addWidget(arm_button)
            disarm_button = QPushButton("上锁")
            disarm_button.clicked.connect(self.disarm_vehicle)
            arm_layout.addWidget(disarm_button)
            side.addLayout(arm_layout)
            altitude_layout = QHBoxLayout()
            altitude_layout.addWidget(QLabel("起飞高度 (m)"))
            self.takeoff_altitude = QDoubleSpinBox()
            self.takeoff_altitude.setRange(float(flight_control.get("min_altitude_m", 1.5)), float(flight_control.get("max_altitude_m", 20.0)))
            self.takeoff_altitude.setDecimals(1)
            self.takeoff_altitude.setSingleStep(0.5)
            self.takeoff_altitude.setValue(float(config.get("grid", {}).get("altitude_m", 2.0)))
            altitude_layout.addWidget(self.takeoff_altitude)
            side.addLayout(altitude_layout)
            takeoff_button = QPushButton("起飞")
            takeoff_button.clicked.connect(self.takeoff_vehicle)
            side.addWidget(takeoff_button)
        side.addStretch(1)
        map_layout.addLayout(side, 1)
        layout.addLayout(map_layout)
        self.setCentralWidget(root)

        self.connect_ros()

    def connect_ros(self):
        if rospy is None:
            self.status_label.setText(self.status_label.text() + " | rospy unavailable (view-only)")
            return
        try:
            if not rospy.core.is_initialized():
                rospy.init_node("ground_station_gui", anonymous=True, disable_signals=True)
            topics = self.config.get("ros", {}).get("topics", {})
            self.command_publisher = rospy.Publisher(topics.get("command", "/uav/ground_station_command"), String, queue_size=10)
            self.status_label.setText(self.status_label.text().replace("disconnected", "ROS connected"))
        except Exception as error:
            self.status_label.setText(self.status_label.text() + " | ROS error: {}".format(error))

    def apply_no_fly_cells(self):
        raw = self.no_fly_edit.text().strip()
        try:
            add_codes = []
            remove_codes = []
            if raw:
                for token in [value.strip() for value in raw.split(",") if value.strip()]:
                    if token.startswith("-"):
                        remove_codes.append(token[1:])
                    elif token.startswith("+"):
                        add_codes.append(token[1:])
                    else:
                        add_codes.append(token)
                self.mission.update_no_fly_cells(add_codes, remove_codes)
            else:
                self.mission.no_fly_cells = set()
            self.mission.route = []
            self.grid_widget.update()
            self.task_status.setText("禁飞区已更新，请重新生成航线")
            self.route_info.setText("航点：0")
            return True
        except ValueError as error:
            QMessageBox.warning(self, "禁飞区无效", str(error))
            return False

    def clear_no_fly_cells(self):
        self.mission.no_fly_cells = set()
        self.mission.route = []
        self.no_fly_edit.clear()
        self.grid_widget.update()
        self.task_status.setText("禁飞区已清空")
        self.route_info.setText("航点：0")

    def plan_route(self):
        if not self.apply_no_fly_cells():
            return
        try:
            route = self.mission.generate_route()
            self.grid_widget.update()
            self.route_info.setText("航点：{}".format(len(route)))
            self.task_status.setText("规划完成")
        except ValueError as error:
            QMessageBox.warning(self, "航线无法生成", str(error))

    def send_command(self, command, payload=None):
        if self.command_publisher is None:
            QMessageBox.warning(self, "通信不可用", "ROS command topic 尚未连接，命令未发送。")
            return
        if not self._wait_for_command_subscriber():
            QMessageBox.warning(self, "通信不可用", "飞控控制节点尚未订阅命令话题，命令未发送。")
            return
        if command == "prepare_mission" and not self.mission.route:
            QMessageBox.warning(self, "任务未规划", "请先生成航线。")
            return
        if payload is None:
            payload = {}
        if command == "prepare_mission":
            altitude = self.takeoff_altitude.value() if self.flight_controls_enabled else self.config["grid"].get("altitude_m", 1.5)
            payload["route"] = [
                {"x": x, "y": y, "z": altitude}
                for x, y in (self.mission.cell_to_local(point) for point in self.mission.route)
            ]
            payload["cell_size_m"] = 1.0
        self.command_publisher.publish(String(data=json.dumps({"command": command, "payload": payload})))

    def _wait_for_command_subscriber(self):
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if self.command_publisher.get_num_connections() > 0:
                return True
            QApplication.processEvents()
            time.sleep(0.05)
        return False

    def set_flight_mode(self):
        mode = self.mode_selector.currentData()
        self.send_command("set_mode", {"mode": mode})

    def arm_vehicle(self):
        confirmation = QMessageBox.question(self, "确认解锁", "确认在 SITL 仿真中解锁无人机？")
        if confirmation == QMessageBox.Yes:
            self.send_command("arm")

    def disarm_vehicle(self):
        confirmation = QMessageBox.question(self, "确认上锁", "仅在无人机已着陆且处于安全状态时上锁。确认继续？")
        if confirmation == QMessageBox.Yes:
            self.send_command("disarm")

    def takeoff_vehicle(self):
        altitude = self.takeoff_altitude.value()
        confirmation = QMessageBox.question(self, "确认起飞", "确认以 {:.1f} 米相对高度起飞？如当前未解锁，系统会先尝试解锁。".format(altitude))
        if confirmation == QMessageBox.Yes:
            self.send_command("takeoff", {"altitude_m": altitude})

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_M:
            self.plan_route()
        else:
            super().keyPressEvent(event)

    def closeEvent(self, event):
        event.accept()


def main():
    if rospy is not None:
        rospy.init_node("ground_station_gui", anonymous=True, disable_signals=True)
        config_path = rospy.get_param("~config_file", "")
    else:
        config_path = os.environ.get("GROUND_STATION_CONFIG", "")
    if not config_path:
        raise RuntimeError("Set ~config_file using roslaunch or GROUND_STATION_CONFIG")
    app = QApplication(sys.argv)
    window = GroundStationWindow(load_config(os.path.expanduser(config_path)))
    window.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()