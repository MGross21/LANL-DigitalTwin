import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetModelState
from std_msgs.msg import Float64MultiArray

class GazeboObjectState(Node):
    def __init__(self):
        super().__init__('gazebo_object_state_node')
        self.client = self.create_client(GetModelState, '/gazebo/get_model_state')
        self.publisher = self.create_publisher(Float64MultiArray, '/object_location', 10)

        # Wait until the service is available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/get_model_state service...')

        # Request and publish the object state periodically
        self.timer = self.create_timer(0.5, self.request_and_publish_state)

    def request_and_publish_state(self):
        request = GetModelState.Request()
        request.model_name = 'april_tag_cube'  # Replace with your model name in Gazebo

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            position = future.result().pose.position
            self.get_logger().info(f'Object position: x={position.x}, y={position.y}, z={position.z}')

            # Publish the position as a Float64MultiArray
            msg = Float64MultiArray()
            msg.data = [position.x, position.y, position.z]
            self.publisher.publish(msg)
        else:
            self.get_logger().error('Failed to get object state.')

def main(args=None):
    rclpy.init(args=args)
    node = GazeboObjectState()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()