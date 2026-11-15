import unittest
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from ground_station.models import GridMission

try:
    from ground_station_node import SitlFlightController
except ImportError:
    SitlFlightController = None


class GridMissionTest(unittest.TestCase):
    def test_route_starts_at_h7_and_ends_at_a1(self):
        mission = GridMission(8, 7)
        route = mission.generate_route()
        self.assertEqual(mission.parse_cell("H7"), route[0])
        self.assertEqual(mission.parse_cell("A1"), route[-1])
        self.assertEqual(56, len(set(route)))
        for first, second in zip(route, route[1:]):
            self.assertEqual(1, abs(first[0] - second[0]) + abs(first[1] - second[1]))

    def test_cell_centers_use_h7_as_origin(self):
        mission = GridMission(8, 7)
        self.assertEqual((0.0, 0.0), mission.cell_to_local(mission.parse_cell("H7")))
        self.assertEqual((0.0, 1.0), mission.cell_to_local(mission.parse_cell("G7")))
        self.assertEqual((1.0, 0.0), mission.cell_to_local(mission.parse_cell("H6")))

    def test_route_covers_all_non_restricted_cells(self):
        mission = GridMission(9, 7)
        mission.set_no_fly_cells(["C2", "C3", "C4", "F6"])
        route = mission.generate_route()
        self.assertEqual(59, len(set(route)))
        self.assertFalse(set(route) & mission.no_fly_cells)
        for first, second in zip(route, route[1:]):
            self.assertEqual(1, abs(first[0] - second[0]) + abs(first[1] - second[1]))

    def test_route_does_not_jump_across_no_fly_cells(self):
        mission = GridMission(3, 5)
        mission.set_no_fly_cells(["B2", "B3", "B4"])
        route = mission.generate_route()
        self.assertEqual(12, len(set(route)))
        self.assertFalse(set(route) & mission.no_fly_cells)
        for first, second in zip(route, route[1:]):
            self.assertEqual(1, abs(first[0] - second[0]) + abs(first[1] - second[1]))

    def test_allows_arbitrary_keep_out_cells(self):
        mission = GridMission()
        mission.set_no_fly_cells(["A1", "B5", "G7", "F3"])
        self.assertEqual({mission.parse_cell("A1"), mission.parse_cell("B5"), mission.parse_cell("G7"), mission.parse_cell("F3")}, mission.no_fly_cells)

    def test_update_no_fly_cells_accumulates_and_removes(self):
        mission = GridMission()
        mission.set_no_fly_cells(["A1", "A2"])
        mission.update_no_fly_cells(["B3"], ["A1"])
        self.assertEqual({mission.parse_cell("A2"), mission.parse_cell("B3")}, mission.no_fly_cells)

    def test_strict_three_cell_mode_still_rejects_non_contiguous_keep_out_cells(self):
        mission = GridMission()
        with self.assertRaises(ValueError):
            mission.set_no_fly_cells(["A1", "A2", "A4"], require_three=True)

    def test_cell_round_trip(self):
        mission = GridMission()
        self.assertEqual("H7", mission.format_cell(mission.parse_cell("h7")))
        with self.assertRaises(ValueError):
            mission.parse_cell("i7")


@unittest.skipIf(SitlFlightController is None, "flight controller script is not on PYTHONPATH")
class SitlFlightControllerTest(unittest.TestCase):
    def setUp(self):
        self.rospy_patcher = patch("ground_station_node.rospy")
        self.rospy = self.rospy_patcher.start()
        self.rospy.is_shutdown.return_value = False
        self.config = {
            "mavros_control": {
                "min_altitude_m": 1.5,
                "max_altitude_m": 20.0,
                "state_timeout_s": 0.1,
            },
            "ros": {"topics": {"command": "/uav/ground_station_command"}},
        }
        with patch.dict("sys.modules", {
            "mavros_msgs": Mock(),
            "mavros_msgs.msg": Mock(State=Mock()),
            "mavros_msgs.srv": Mock(CommandBool=Mock(), CommandTOL=Mock(), SetMode=Mock()),
        }):
            self.controller = SitlFlightController(self.config)
        self.controller.state = SimpleNamespace(connected=True, armed=False, mode="MANUAL")
        self.controller.state_subscriber = Mock()
        self.controller.arm_service = Mock(return_value=SimpleNamespace(success=True))
        self.controller.takeoff_service = Mock(return_value=SimpleNamespace(success=True))
        self.controller.mode_service = Mock(return_value=SimpleNamespace(mode_sent=True))

    def tearDown(self):
        self.rospy_patcher.stop()

    def test_arm_waits_for_armed_state(self):
        self.controller.state.armed = True
        with patch("ground_station_node.time.sleep"):
            self.controller._command_callback(SimpleNamespace(data='{"command":"arm"}'))
        self.controller.arm_service.assert_called_once_with(True)

    def test_takeoff_arms_and_calls_takeoff_service_without_mode_switch(self):
        def arm(_value):
            self.controller.state.armed = True
            return SimpleNamespace(success=True)

        self.controller.arm_service.side_effect = arm
        with patch("ground_station_node.time.sleep"):
            self.controller._command_callback(SimpleNamespace(data='{"command":"takeoff","payload":{"altitude_m":3.0}}'))
        self.controller.takeoff_service.assert_called_once_with(
            min_pitch=0.0, yaw=0.0, latitude=0.0, longitude=0.0, altitude=3.0
        )
        self.controller.mode_service.assert_not_called()

    def test_takeoff_does_not_call_service_when_arming_is_rejected(self):
        self.controller.arm_service.return_value = SimpleNamespace(success=False)
        self.controller._command_callback(SimpleNamespace(data='{"command":"takeoff","payload":{"altitude_m":3.0}}'))
        self.controller.takeoff_service.assert_not_called()

    def test_map_coordinates_are_converted_to_mavros_world_coordinates(self):
        self.assertEqual((-1.0, 0.0, 2.5), self.controller._map_to_mavros_position((1.0, 0.0, 2.5)))
        self.assertEqual((0.0, 1.0, 2.5), self.controller._map_to_mavros_position((0.0, 1.0, 2.5)))

    def test_setpoint_orientation_keeps_heading_toward_positive_y(self):
        pose = SimpleNamespace(pose=SimpleNamespace(orientation=SimpleNamespace(x=None, y=None, z=None, w=None)))
        self.controller._setpoint_orientation(pose)
        self.assertAlmostEqual(0.0, pose.pose.orientation.x)
        self.assertAlmostEqual(0.0, pose.pose.orientation.y)
        self.assertAlmostEqual(2 ** -0.5, pose.pose.orientation.z)
        self.assertAlmostEqual(2 ** -0.5, pose.pose.orientation.w)

    def test_final_waypoint_requests_auto_land_once(self):
        self.controller.route = [(0.0, 0.0, 2.5)]
        self.controller.position = (0.0, 0.0, 2.5)
        self.controller._advance_waypoint_if_reached()
        self.controller._advance_waypoint_if_reached()
        self.controller.mode_service.assert_called_once_with(base_mode=0, custom_mode="AUTO.LAND")
        self.assertTrue(self.controller.landing_requested.is_set())

    def test_intermediate_waypoint_does_not_request_auto_land(self):
        self.controller.route = [(0.0, 0.0, 2.5), (1.0, 0.0, 2.5)]
        self.controller.position = (0.0, 0.0, 2.5)
        self.controller._advance_waypoint_if_reached()
        self.controller.mode_service.assert_not_called()
        self.assertEqual(1, self.controller.route_index)


if __name__ == "__main__":
    unittest.main()