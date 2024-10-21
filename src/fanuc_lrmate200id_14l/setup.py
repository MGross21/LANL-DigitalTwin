from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'fanuc_lrmate200id_14l'

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
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS2 package for Fanuc LR Mate 200iD/14L robot',
    license='BSD-3-Clause',  # Update this to match your actual license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fanuc_node = fanuc_lrmate200id_14l.fanuc_node:main'
            'set_joint_state = fanuc_lrmate200id_14l.set_joint_state:main',  # Adjusted to be in the scripts folder
        ],
    },
)