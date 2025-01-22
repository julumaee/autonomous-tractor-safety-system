
######################################################################################
### NOTE: This could be more time-efficient by a multithreaded implementation NOTE ###
######################################################################################

import rclpy
from rclpy.node import Node
from tractor_safety_system_interfaces.msg import FusedDetection, ControlCommand

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
        self.safety_distance_1 = 40  # A distance threshold for controlling safety
        self.safety_distance_2 = 10  # A distance threshold for controlling safety
        self.safety_distance_3 = 3   # A distance threshold for controlling safety
        self.speed_override_1  = 5   # A value for overriding tractor speed with if a target is inside safety_distance_1
        self.speed_override_2  = 2   # A value for overriding tractor speed with if a target is inside safety_distance_2
        self.speed_override_3  = 0   # Stop the tractor if a target is inside safety_distance_3
        
        self.active_detection = False                       # By default no detections percepted
        self.speed_to_override = 0                          # A variable for overriding speed
        self.latest_steering_angle = 0                      # Default steering angle from AgOpenGPS
        self.last_detection_time = self.get_clock().now()   # Save the time of last detection
        
    def classify_detection(self, detection):
        distance = detection.distance
        self.get_logger().info(f"Received detection: {detection.header.frame_id} at distance: {distance}m")

        # Override control commands based on safety logic
        if distance <= self.safety_distance_3:
            self.active_detection = True
            stop_cmd = ControlCommand()
            stop_cmd.speed = self.speed_override_3
            stop_cmd.steering_angle = self.latest_steering_angle
            self.publisher_.publish(stop_cmd) # Stop
            self.get_logger().info(f"Stopped, when a {detection.header.frame_id} was in distance {distance}m")
        elif distance <= self.safety_distance_2:
            self.speed_to_override = self.speed_override_2 # Slow speed
            self.active_detection = True
        elif distance <= self.safety_distance_1:
            self.speed_to_override = self.speed_override_1 # Moderate speed
            self.active_detection = True
        else:
            self.active_detection = False  # No active detections in range TODO add some timer here?
        self.last_detection_time = self.get_clock().now()


    def agopen_control(self, agopen_cmd):
        # Reset active_detection if nothing detected for more than 5 seconds
        time_diff = (self.get_clock().now() - self.last_detection_time).nanoseconds / 1e9
        if time_diff > 5.0:
            self.active_detection = False
        if not self.active_detection:
            # Forward AgOpenGPS commands if no active detections
            self.get_logger().info("No active detections, forwarding AgOpenGPS control commands.")
            self.publisher_.publish(agopen_cmd)
            self.latest_steering_angle = agopen_cmd.steering_angle
        else:
            self.get_logger().info(f"Active detection present, overriding AgOpenGPS speed command to {self.speed_to_override}.")
            agopen_cmd.speed = self.speed_to_override
            self.publisher_.publish(agopen_cmd)
            self.latest_steering_angle = agopen_cmd.steering_angle

def main(args=None):
    rclpy.init(args=args)
    safety_monitor = SafetyMonitor()
    rclpy.spin(safety_monitor)
    safety_monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()