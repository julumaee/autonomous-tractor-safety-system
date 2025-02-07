import random

from geometry_msgs.msg import Point
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from tractor_safety_system_interfaces.msg import SimulatedObject


class ObjectSimulator(Node):

    def __init__(self):
        super().__init__('object_simulator')
        self.publisher_ = self.create_publisher(SimulatedObject, '/simulated_objects', 10)
        self.timer = self.create_timer(2, self.simulate_detection)

        # Declare parameters with default values
        self.declare_parameter('min_x', 0)
        self.declare_parameter('min_y', -10)
        self.declare_parameter('min_z', 0)
        self.declare_parameter('max_x', 50)
        self.declare_parameter('max_y', 10)
        self.declare_parameter('max_z', 0)

        # Retrieve the parameter values
        self.min_x = self.get_parameter('min_x').value
        self.min_y = self.get_parameter('min_y').value
        self.min_z = self.get_parameter('min_z').value
        self.max_x = self.get_parameter('max_x').value
        self.max_y = self.get_parameter('max_y').value
        self.max_z = self.get_parameter('max_z').value

        # Subscribe to parameter updates
        self.add_on_set_parameters_callback(self.on_set_parameters)

    def on_set_parameters(self, params):
        """Set parameters their new values."""
        for param in params:
            if param.name == 'min_x':
                self.min_x = param.value
            elif param.name == 'min_y':
                self.min_y = param.value
            elif param.name == 'min_z':
                self.min_z = param.value
            elif param.name == 'max_x':
                self.max_x = param.value
            elif param.name == 'max_y':
                self.max_y = param.value
            elif param.name == 'max_z':
                self.max_z = param.value
        return SetParametersResult(successful=True)

    def simulate_detection(self):
        """Generate simulated object data."""
        simulated_object = SimulatedObject()
        simulated_object.object_id = random.randint(0, 255)  # Object ID: 0-255
        simulated_object.position = self.generate_position()
        self.publisher_.publish(simulated_object)
        self.get_logger().info(f'Published a simulated object with ID=\
                               {simulated_object.object_id}')

    def generate_position(self):
        """Generate a random 3D position."""
        position = Point()
        position.x = random.uniform(self.min_x, self.max_x)
        position.y = random.uniform(self.min_y, self.max_y)
        position.z = random.uniform(self.min_z, self.max_z)
        return position


def main(args=None):
    rclpy.init(args=args)
    object_simulator = ObjectSimulator()
    rclpy.spin(object_simulator)
    object_simulator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
