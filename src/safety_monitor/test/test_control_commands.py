##############################################################################################
#                                                                                            #
# This module contains a set of unit tests for the SafetyMonitor class, which is part of the #
# tractor safety system. The tests are designed to ensure that the SafetyMonitor correctly   #
# processes and overrides control commands based on the current state.                      #
#                                                                                            #
# Classes:                                                                                   #
#    TestFusionNode: A unittest.TestCase subclass that contains tests for the SafetyMonitor  #
#    class.                                                                                  #
#                                                                                            #
# Methods:                                                                                   #
#    setUpClass(cls): Initializes ROS2 before any test is run.                               #
#    tearDownClass(cls): Shuts down ROS2 after all tests are completed.                      #
#    setUp(self): Initializes a SafetyMonitor instance with a mocked publisher.              #
#    create_agopen_command(self, speed, steering_angle): Creates a ControlCommand message.   #
#    test_safety_monitor_speed_override(self): Ensures that speed overrides are correctly    #
#    applied based on the current state of the SafetyMonitor.                                #
#                                                                                            #
##############################################################################################

import unittest

import rclpy
from safety_monitor.safety_monitor import SafetyMonitor
from tractor_safety_system_interfaces.msg import ControlCommand


class TestSafetyMonitorCommands(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2 before any test is run."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2 after all tests are completed."""
        rclpy.shutdown()

    def setUp(self):
        """Initialize the FusionNode for testing."""
        self.safety_monitor = SafetyMonitor()

        # Mock publisher output
        self.published_commands = []

        def mock_publisher(control_cmd):
            """Mock method to capture published messages."""
            self.published_commands.append(control_cmd)

        self.safety_monitor.publisher_.publish = mock_publisher

    def create_agopen_command(self, speed, steering_angle):
        """Create a ControlCommand message with the specified speed and steering angle."""
        control_cmd = ControlCommand()
        control_cmd.speed = speed
        control_cmd.steering_angle = steering_angle
        return control_cmd

    def test_safety_monitor_speed_override(self):
        """Ensure that the safety monitor states transition correctly."""
        self.safety_monitor.speed_override_1 = 5
        self.safety_monitor.speed_override_2 = 2
        control_cmd = self.create_agopen_command(10, 0)

        # Test overriding speed when in state 'moderate'
        self.safety_monitor.vehicle_state = 'moderate'
        self.safety_monitor.agopen_control(control_cmd)
        self.assertEqual(len(self.published_commands), 1,
                         msg='Control command not published in state moderate')
        self.assertEqual(self.published_commands[0].speed,
                         self.safety_monitor.speed_override_1,
                         msg='Speed override 1 not applied in state moderate')

        # Test overriding speed when in state 'slow'
        self.safety_monitor.vehicle_state = 'slow'
        self.safety_monitor.agopen_control(control_cmd)
        self.assertEqual(len(self.published_commands), 2,
                         msg='Control command not published in state slow')
        self.assertEqual(self.published_commands[1].speed,
                         self.safety_monitor.speed_override_2,
                         msg='Speed override 2 not applied in state slow')

        # Test overriding agopen commands when in state 'stopped'
        self.safety_monitor.vehicle_state = 'stopped'
        self.safety_monitor.agopen_control(control_cmd)
        self.assertEqual(len(self.published_commands), 2,
                         msg='Control commands should not be published in state stopped')

        # Test overrriding agopen commands when in unknown state
        self.safety_monitor.vehicle_state = 'anything'
        self.safety_monitor.agopen_control(control_cmd)
        self.assertEqual(len(self.published_commands), 3,
                         msg='Stop command not published in unknown state')
        self.assertEqual(self.published_commands[2].speed, 0,
                         msg='Vehicle should be stopped in unknown state')


if __name__ == '__main__':
    unittest.main()
