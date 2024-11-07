from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'robo_ag_rl'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'gazebo_ros',
        # Add any other Python package dependencies here
    ],
    zip_safe=True,
    maintainer='Michael Gross',
    maintainer_email='mhgross2@asu.edu',
    description='ROS2 package for Agnostic Robotic System',
    license='BSD-3-Clause',  # Update this to match your actual license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'robopos = {package_name}.robopos_node:main'
            f'set_joint_state = {package_name}.set_joint_states:main',  # Adjusted to be in the scripts folder
        ],
    },
)