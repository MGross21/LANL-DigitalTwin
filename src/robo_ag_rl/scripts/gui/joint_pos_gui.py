import sys
import rclpy
from rclpy.node import Node
from PyQt6.QtWidgets import QApplication, QSlider, QLabel, QVBoxLayout, QWidget, QHBoxLayout, QFrame
from PyQt6.QtCore import Qt, QTimer
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import numpy as np
import datetime

class JointStatePublisherGUI(Node):
    def __init__(self):
        super().__init__('joint_state_publisher_gui')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)

        # Initialize joints dynamically based on a parameter or input
        self.declare_parameter('joint_names', ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'])
        self.joint_names = self.get_parameter('joint_names').get_parameter_value().string_array_value
        self.joint_positions = [0.0] * len(self.joint_names)

        # Set up GUI
        self.app = QApplication(sys.argv)
        self.window = QWidget()
        self.window.setWindowTitle('Joint Position Controller')
        self.main_layout = QVBoxLayout()

        # Create a frame for sliders
        slider_frame = QFrame()
        slider_layout = QVBoxLayout(slider_frame)

        # Create sliders and labels for each joint
        self.sliders = []
        self.labels = []
        for i, joint_name in enumerate(self.joint_names):
            label = QLabel(f'{joint_name}: {self.joint_positions[i]:.2f}')
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setMinimum(-314)
            slider.setMaximum(314)
            slider.setValue(0)
            slider.valueChanged.connect(lambda value, idx=i: self.update_joint_position(value, idx))

            slider_layout.addWidget(label)
            slider_layout.addWidget(slider)
            self.sliders.append(slider)
            self.labels.append(label)

        self.main_layout.addWidget(slider_frame)

        # Set up the real-time plot
        self.figure, self.ax = plt.subplots()
        self.canvas = FigureCanvas(self.figure)
        self.main_layout.addWidget(self.canvas)
        self.time_data = []
        self.joint_data = {joint_name: [] for joint_name in self.joint_names}
        
        self.window.setLayout(self.main_layout)

        # Start a timer to periodically update the plot and publish data
        self.timer = self.create_timer(0.1, self.publish_joint_states)
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.update_plot)
        self.plot_timer.start(100)

        self.window.show()

    def update_joint_position(self, value, joint_index):
        self.joint_positions[joint_index] = value / 100.0
        self.labels[joint_index].setText(f'{self.joint_names[joint_index]}: {self.joint_positions[joint_index]:.2f}')

    def publish_joint_states(self):
        joint_state_msg = JointState()
        joint_state_msg.name = self.joint_names
        joint_state_msg.position = self.joint_positions
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(joint_state_msg)

        # Update data for plotting
        current_time = datetime.datetime.now()
        self.time_data.append(current_time)
        if len(self.time_data) > 100:  # Keep only the last 10 seconds of data (assuming 10 Hz)
            self.time_data = self.time_data[-100:]

        for i, joint_name in enumerate(self.joint_names):
            self.joint_data[joint_name].append(self.joint_positions[i])
            if len(self.joint_data[joint_name]) > 100:
                self.joint_data[joint_name] = self.joint_data[joint_name][-100:]

    def update_plot(self):
        self.ax.clear()
        for joint_name in self.joint_names:
            self.ax.plot(self.time_data, self.joint_data[joint_name], label=joint_name)

        self.ax.set_title('Joint Angles Over Time')
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Angle (radians)')
        self.ax.legend()
        self.figure.autofmt_xdate()
        self.canvas.draw()

def main(args=None):
    rclpy.init(args=args)
    node = JointStatePublisherGUI()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
