import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import numpy as np

class MockObservationPublisher(Node):
    def __init__(self):
        super().__init__('mock_observation_publisher')
        self.publisher_ = self.create_publisher(String, 'observations', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('Mock Observation Publisher Started')

    def timer_callback(self):
        observation = {
            'x': float(np.random.uniform(-0.1, 0.2)),
            'y': float(np.random.uniform(-0.2, 0.2)),
            'z': float(np.random.uniform(-0.4, 0.4)),
            'theta': float(np.random.uniform(-1.0, 4.0)),
            'vx': float(np.random.uniform(-0.2, 0.1)),
            'vy': float(np.random.uniform(-0.1, 0.1)),
            'vz': float(np.random.uniform(-0.3, 0.3)),
            'omega': float(np.random.uniform(-0.5, 0.7)),
            'target_x': float(np.random.uniform(-0.1, 0.2)),
            'target_y': float(np.random.uniform(-0.2, 0.2)),
            'target_z': float(np.random.uniform(-0.5, 0.5)),
            'target_angle': float(np.random.uniform(-4.5, 1.5))
        }

        msg = String()
        msg.data = json.dumps(observation)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published observation: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = MockObservationPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()