import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from rclpy.time import Time

class RobotEnv:
    def __init__(self):
        rclpy.init()
        self.node = Node("robot_env_node")

        # Set up ROS publishers and subscribers
        self.joint_state_subscriber = self.node.create_subscription(
            JointState,
            '/my_robot/joint_states',  # Update with your robot's joint state topic
            self.joint_state_callback,
            10
        )
        self.action_publisher = self.node.create_publisher(
            Float64MultiArray,
            '/my_robot/set_joint_speeds',  # Update with your robot's action topic
            10
        )

        # Initialize environment variables
        self.previous_joint_state = np.zeros(6)  # Assuming a 6-DOF robot; update as needed
        self.current_time = 0.0  # Keep track of time in seconds
        self.joint_limits = np.array([[-np.pi, np.pi]] * 6)  # Joint limits
        self.object_grasped = False  # Flag for object grasping

    def joint_state_callback(self, msg):
        # Update the current joint state based on the incoming message
        self.previous_joint_state = np.array(msg.position)

    def calculate_reward(self, state, action):
        # Initialize total reward
        total_reward = 0.0

        # Joint Speed Control Reward: Reward for moving joints within limits and reducing speed
        for i in range(len(action)):
            if self.joint_limits[i, 0] <= state[i] <= self.joint_limits[i, 1]:
                total_reward += 1.0  # Reward for staying within limits
            total_reward -= abs(action[i])  # Penalty for excessive speed

        # Delta Time Penalty
        if self.current_time > 15.0:
            total_reward -= 1.0  # Penalty if time exceeds limit

        return total_reward

    def update(self, state, action):
        # Update the environment with the current state and action
        self.previous_action = action.copy()

        # Publish the action to the robot's joint controllers as speeds
        msg = Float64MultiArray()
        msg.data = action.tolist()
        self.action_publisher.publish(msg)

        # Update current time using Gazebo sim time or system time
        sim_time = self.get_sim_time()
        self.current_time = sim_time

        # Calculate and return the reward
        return self.calculate_reward(state, action)

    def get_sim_time(self):
        # Retrieve simulation time from Gazebo or use system time
        current_time = self.node.get_clock().now().to_msg()  # Get current ROS time
        return current_time.secs + current_time.nsecs / 1e9  # Convert to seconds

    def render(self, mode='human'):
        # Optional rendering method
        pass

    def close(self):
        self.node.destroy_node()
        rclpy.shutdown()
