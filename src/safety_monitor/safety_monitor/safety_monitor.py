# Copyright 2025 Eemil Kulmala, University of Oulu
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from tractor_safety_system_interfaces.msg import ControlCommand, FusedDetection


class SafetyMonitor(Node):

    def __init__(self):
        super().__init__('safety_monitor')
        self.publisher_ = self.create_publisher(ControlCommand, '/control', 10)
        self.detection_subscription = self.create_subscription(
            FusedDetection,
            '/fused_detections',
            self.control_speed_state,
            10)
        self.agopen_subscription = self.create_subscription(
            ControlCommand,
            '/control/agopen',
            self.agopen_control,
            10)

        # Control the vehicle state every 0.1 seconds
        self.timer = self.create_timer(0.1, self.state_control)

        # Declare parameters with default values
        self.declare_parameter('safety_distance_1', 40)
        self.declare_parameter('safety_distance_2', 10)
        self.declare_parameter('stop_distance', 3)
        self.declare_parameter('speed_override_1', 5)
        self.declare_parameter('speed_override_2', 2)
        self.declare_parameter('detection_active_reset_time', 5.0)
        self.declare_parameter('vehicle_stopped_reset_time', 5.0)

        # Retrieve the parameter values
        self.safety_distance_1 = self.get_parameter('safety_distance_1').value
        self.safety_distance_2 = self.get_parameter('safety_distance_2').value
        self.stop_distance = self.get_parameter('stop_distance').value
        self.speed_override_1 = self.get_parameter('speed_override_1').value
        self.speed_override_2 = self.get_parameter('speed_override_2').value
        self.detection_active_reset_time = self.get_parameter('detection_active_reset_time').value
        self.vehicle_stopped_reset_time = self.get_parameter('vehicle_stopped_reset_time').value

        # Subscribe to parameter updates
        self.add_on_set_parameters_callback(self.on_set_parameters)

        # Initialize variables
        self.vehicle_state = 'agopen'   # Vehicle state: 'agopen, moderate, slow or stopped'
        self.latest_steering_angle = 0  # Latest steering angle from AgOpenGPS
        self.agopen_speed = 0           # Latest speed from AgOpenGPS
        self.stop_time = self.get_clock().now()  # Save the time when vehicle stopped

        # Time of last detection inside safety_distance_1
        self.last_detection_time_1 = self.get_clock().now()

        # Time of last detection inside safety_distance_2
        self.last_detection_time_2 = self.get_clock().now()

    def on_set_parameters(self, params):
        """Set parameters their new values."""
        for param in params:
            if param.name == 'safety_distance_1':
                self.safety_distance_1 = param.value
            elif param.name == 'safety_distance_2':
                self.safety_distance_2 = param.value
            elif param.name == 'stop_distance':
                self.stop_distance = param.value
            elif param.name == 'speed_override_1':
                self.speed_override_1 = param.value
            elif param.name == 'speed_override_2':
                self.speed_override_2 = param.value
            elif param.name == 'detection_active_reset_time':
                self.detection_active_reset_time = param.value
            elif param.name == 'vehicle_stopped_reset_time':
                self.vehicle_stopped_reset_time = param.value
        return SetParametersResult(successful=True)

    def agopen_control(self, agopen_cmd):
        """Process AgOpenGPS control commands."""
        self.latest_steering_angle = agopen_cmd.steering_angle
        self.agopen_speed = agopen_cmd.speed
        self.speed_control()

    def control_speed_state(self, detection):
        """Control the tractor speed state based on detection distances."""
        distance = detection.distance
        # self.get_logger().info('Received detection: '
        #                        f'{detection.header.frame_id} at distance: {distance}m')

        # Modify the vehicle state based on received target distance
        if distance <= self.stop_distance:
            if self.vehicle_state != 'stopped':
                self.get_logger().info(f'Obstacle at {distance}m! Stopping the vehicle.')
                self.send_stop_command()
                self.vehicle_state = 'stopped'
            self.stop_time = self.get_clock().now()

        elif distance <= self.safety_distance_2:
            self.last_detection_time_2 = self.get_clock().now()

            if (self.vehicle_state != 'stopped'):
                self.vehicle_state = 'slow'

        elif distance <= self.safety_distance_1:
            self.last_detection_time_1 = self.get_clock().now()

            if self.vehicle_state not in ['stopped', 'slow']:
                self.vehicle_state = 'moderate'

    def send_stop_command(self):
        """Send a stop command."""
        stop_cmd = ControlCommand()
        stop_cmd.speed = 0
        stop_cmd.steering_angle = self.latest_steering_angle
        self.publisher_.publish(stop_cmd)

    def state_control(self):
        """Update speed state if nothing detected during reset time."""
        current_time = self.get_clock().now()
        time_diff_1 = (current_time - self.last_detection_time_1).nanoseconds / 1e9
        time_diff_2 = (current_time - self.last_detection_time_2).nanoseconds / 1e9
        if self.vehicle_state == 'stopped':
            if ((self.get_clock().now() -
                    self.stop_time).nanoseconds /
                    1e9 > self.vehicle_stopped_reset_time):
                self.vehicle_state = 'slow'
                self.last_detection_time_2 = current_time  # Reset timer to delay speed change
                self.get_logger().info('Vehicle has been stopped for '
                                       f'{self.vehicle_stopped_reset_time} seconds. '
                                       'Changing state to "slow".')
        elif self.vehicle_state == 'slow':
            if time_diff_2 > self.detection_active_reset_time:
                self.vehicle_state = 'moderate'
                self.last_detection_time_1 = current_time  # Reset timer to delay speed change
                self.get_logger().info('No detections inside safety_distance_2 '
                                       f'for {self.detection_active_reset_time} seconds. '
                                       'Changing state to "moderate".')
        elif self.vehicle_state == 'moderate':
            if time_diff_1 > self.detection_active_reset_time:
                self.vehicle_state = 'agopen'
                self.get_logger().info('No detections inside safety_distance_1 '
                                       f'for {self.detection_active_reset_time} seconds. '
                                       'Changing state to "agopen".')
        elif self.vehicle_state == 'agopen':
            pass
        else:
            self.vehicle_state = 'stopped'
            self.speed_control()
            self.stop_time = self.get_clock().now()
            self.get_logger().info('Vehicle in unknown state. Stopping the vehicle.')

    def speed_control(self):
        """Override agopen control commands based on the speed state."""
        control_cmd = ControlCommand()
        control_cmd.steering_angle = self.latest_steering_angle

        if self.vehicle_state == 'agopen':
            # Forward AgOpenGPS commands if no active detections
            control_cmd.speed = self.agopen_speed
            self.get_logger().info('No active detections, forwarding AgOpenGPS control commands.')
            self.publisher_.publish(control_cmd)
        elif self.vehicle_state == 'stopped':
            # Publish only reverse commands, when in stop state
            if self.agopen_speed < 0:
                control_cmd.speed = self.agopen_speed
                self.publisher_.publish(control_cmd)
                self.get_logger().info('Vehicle in state "stopped", '
                                       'but AgOpenGPS command is reverse. '
                                       'Forwarding AgOpenGPS control commands.')
            else:
                self.get_logger().info('Vehicle will remain stopped for at least'
                                       f'{self.vehicle_stopped_reset_time -
                                       (self.get_clock().now() -
                                       self.stop_time).nanoseconds / 1e9} seconds.'
                                       ' Ignoring AgOpenGPS forward commands.')
        elif self.vehicle_state == 'slow':
            if (self.agopen_speed > self.speed_override_2):
                control_cmd.speed = self.speed_override_2
                self.get_logger().info('Vehicle in state "slow", '
                                       'overriding AgOpenGPS speed command to '
                                       f'{self.speed_override_2}.')
            else:
                control_cmd.speed = self.agopen_speed
                self.get_logger().info('Vehicle in state "slow", '
                                       f'but agopen speed ( {self.agopen_speed} ) is under '
                                       f'{self.speed_override_2}\
                                       . Forwarding AgOpenGPS control commands.')
            self.publisher_.publish(control_cmd)
        elif self.vehicle_state == 'moderate':
            if (self.agopen_speed > self.speed_override_1):
                control_cmd.speed = self.speed_override_1
                self.get_logger().info('Vehicle in state "moderate" , '
                                       'overriding AgOpenGPS speed command to '
                                       f'{self.speed_override_1}.')
            else:
                control_cmd.speed = self.agopen_speed
                self.get_logger().info('Vehicle in state "moderate", '
                                       f'but agopen speed ( {self.agopen_speed} )'
                                       f'is under {self.speed_override_1}'
                                       '. Forwarding AgOpenGPS control commands.')
            self.publisher_.publish(control_cmd)
        else:
            control_cmd.speed = 0
            self.publisher_.publish(control_cmd)
            self.stop_time = self.get_clock().now()
            self.get_logger().info('Vehicle in unknown state. Stopping the vehicle.')


def main(args=None):
    rclpy.init(args=args)
    safety_monitor = SafetyMonitor()
    rclpy.spin(safety_monitor)
    safety_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
