import rclpy
import random
from rclpy.node import Node
from tractor_safety_system_interfaces.msg import ControlCommand

class AgOpenSimulator(Node):

    def __init__(self):
        super().__init__('agopen_simulator')
        self.publisher_ = self.create_publisher(ControlCommand, '/control/agopen', 10)
        self.timer = self.create_timer(0.5, self.simulate_control)

    def simulate_control(self):
        command = ControlCommand()
        command.speed = random.randint(10, 15)
        command.steering_angle = random.randint(0, 50)
        self.publisher_.publish(command)

def main(args=None):
    rclpy.init(args=args)
    agopen_simulator = AgOpenSimulator()
    rclpy.spin(agopen_simulator)
    agopen_simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()