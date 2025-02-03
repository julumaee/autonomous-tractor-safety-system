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
            self.classify_detection,
            10)
        self.agopen_subscription = self.create_subscription(
            ControlCommand,
            '/control/agopen',
            self.agopen_control,
            10)

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
        self.vehicle_state = 'agopen'  # Vehicle state: 'agopen, moderate, slow or stopped'
        self.latest_steering_angle = 0  # Latest steering angle from AgOpenGPS
        self.last_detection_time = self.get_clock().now()   # Save the time of last detection
        self.stop_time = self.get_clock().now()             # Save the time when vehicle stopped

    def on_set_parameters(self, params):
        """Set parameters their new values."""
        for param in params:
            if param.name == 'safety_distance_1':
                self.safety_distance_1 = param.value
            elif param.name == 'safety_distance_2':
                self.safety_distance_2 = param.value
            elif param.name == 'stop_distance':
                self.safety_distance_3 = param.value
            elif param.name == 'speed_override_1':
                self.speed_override_1 = param.value
            elif param.name == 'speed_override_2':
                self.speed_override_2 = param.value
            elif param.name == 'detection_active_reset_time':
                self.detection_active_reset_time = param.value
            elif param.name == 'vehicle_stopped_reset_time':
                self.vehicle_stopped_reset_time = param.value
        return SetParametersResult(successful=True)

    def classify_detection(self, detection):
        distance = detection.distance
        self.get_logger().info(f'Received detection: \
                               {detection.header.frame_id} at distance: {distance}m')

        # Modify the vehicle state based on received target distance
        if distance <= self.stop_distance:
            self.stop_time = self.get_clock().now()
            self.last_detection_time = self.get_clock().now()

            if self.vehicle_state != 'stopped':
                self.send_stop_command
                self.vehicle_state = 'stopped'

        elif distance <= self.safety_distance_2:
            self.last_detection_time = self.get_clock().now()

            if (self.vehicle_state != 'stopped'):
                self.vehicle_state = 'slow'

        elif distance <= self.safety_distance_1:
            self.last_detection_time = self.get_clock().now()

            if self.vehicle_state not in ['stopped', 'slow']:
                self.vehicle_state = 'moderate'

    def send_stop_command(self):
        stop_cmd = ControlCommand()
        stop_cmd.speed = 0
        stop_cmd.steering_angle = self.latest_steering_angle
        self.publisher_.publish(stop_cmd)

    def agopen_control(self, agopen_cmd):
        # Reset active_detection if nothing detected for more than 5 seconds
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_detection_time).nanoseconds / 1e9
        if self.vehicle_state == 'stopped':
            if ((self.get_clock().now() -
                    self.stop_time).nanoseconds /
                    1e9 > self.vehicle_stopped_reset_time):
                self.vehicle_state = 'slow'
                self.last_detection_time = current_time  # Reset timer to delay speed change
        else:
            if time_diff > self.detection_active_reset_time:
                if (self.vehicle_state == 'slow'):
                    self.vehicle_state = 'moderate'
                    self.last_detection_time = current_time  # Reset timer to delay speed change
                else:
                    self.vehicle_state = 'agopen'

        if self.vehicle_state == 'agopen':
            # Forward AgOpenGPS commands if no active detections
            self.get_logger().info('No active detections, forwarding AgOpenGPS control commands.')
            self.publisher_.publish(agopen_cmd)
            self.latest_steering_angle = agopen_cmd.steering_angle
        elif self.vehicle_state == 'stopped':
            # Publish no commands, when in stop state
            self.get_logger().info(f'Vehicle will remain stopped \
                                   for at least \
                                   {self.vehicle_stopped_reset_time -
                                    (self.get_clock().now() -
                                     self.stop_time).nanoseconds / 1e9} seconds.')
        elif self.vehicle_state == 'slow':
            self.get_logger().info(f'Vehicle in state "slow", \
                                   overriding AgOpenGPS speed command to \
                                   {self.speed_override_2}.')
            agopen_cmd.speed = self.speed_override_2
            self.publisher_.publish(agopen_cmd)
            self.latest_steering_angle = agopen_cmd.steering_angle
        elif self.vehicle_state == 'moderate':
            self.get_logger().info(f'Vehicle in state "moderate" , \
                                   overriding AgOpenGPS speed command to \
                                   {self.speed_override_1}.')
            agopen_cmd.speed = self.speed_override_1
            self.publisher_.publish(agopen_cmd)
            self.latest_steering_angle = agopen_cmd.steering_angle
        else:
            self.send_stop_command
            self.get_logger().info('Vehicle in unknown state. Stopping the vehicle.')


def main(args=None):
    rclpy.init(args=args)
    safety_monitor = SafetyMonitor()
    rclpy.spin(safety_monitor)
    safety_monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
