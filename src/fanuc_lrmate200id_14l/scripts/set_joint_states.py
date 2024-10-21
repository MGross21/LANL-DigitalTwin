import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
        self.timer = self.create_timer(1.0, self.publish_joint_states)

    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.name = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
        ]
        joint_state_msg.position = [
            0.0,  # Joint 1 position
            -0.5,  # Joint 2 position
            0.25,  # Joint 3 position
            1.0,  # Joint 4 position
            -0.75,  # Joint 5 position
            0.5   # Joint 6 position
        ]

        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(joint_state_msg)
        self.get_logger().info('Published joint states to set robot to initial position')

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
