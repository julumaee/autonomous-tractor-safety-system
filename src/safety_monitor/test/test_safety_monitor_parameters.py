import unittest

import rclpy
from rclpy.parameter import Parameter
from safety_monitor.safety_monitor import SafetyMonitor


class TestSafetyMonitorParameters(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 before any test is run."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2 after all tests are completed."""
        rclpy.shutdown()

    def setUp(self):
        """Initialize the SafetyMonitor for testing."""
        self.safety_monitor = SafetyMonitor()

    def test_parameter_update(self):
        """Test that parameters can be updated correctly."""
        new_params = [
            Parameter(name='safety_distance_1', value=50),
            Parameter(name='safety_distance_2', value=20),
            Parameter(name='stop_distance', value=4),
            Parameter(name='speed_override_1', value=6),
            Parameter(name='speed_override_2', value=3),
            Parameter(name='detection_active_reset_time', value=6.0),
            Parameter(name='vehicle_stopped_reset_time', value=6.0)
        ]
        self.safety_monitor.on_set_parameters(new_params)

        # Check that the parameters are updated correctly
        self.assertEqual(self.safety_monitor.safety_distance_1, 50,
                         msg='safety_distance_1 parameter did not update correctly')
        self.assertEqual(self.safety_monitor.safety_distance_2, 20,
                         msg='safety_distance_2 parameter did not update correctly')
        self.assertEqual(self.safety_monitor.stop_distance, 4,
                         msg='stop_distance parameter did not update correctly')
        self.assertEqual(self.safety_monitor.speed_override_1, 6,
                         msg='speed_override_1 parameter did not update correctly')
        self.assertEqual(self.safety_monitor.speed_override_2, 3,
                         msg='speed_override_2 parameter did not update correctly')
        self.assertEqual(self.safety_monitor.detection_active_reset_time, 6.0,
                         msg='detection_active_reset_time parameter did not update correctly')
        self.assertEqual(self.safety_monitor.vehicle_stopped_reset_time, 6.0,
                         msg='vehicle_stopped_reset_time parameter did not update correctly')


if __name__ == '__main__':
    unittest.main()
