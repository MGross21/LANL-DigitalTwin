import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define project package
    project_package = 'robo_ag_rl'
    pkg_share = get_package_share_directory(project_package)

    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    server = LaunchConfiguration('server')
    world_file = LaunchConfiguration('world_file')
    urdf_file = LaunchConfiguration('urdf_file')
    rviz_config = LaunchConfiguration('rviz_config')
    initial_joint_states = LaunchConfiguration('initial_joint_states')

    # Declare launch arguments
    launch_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('gui', default_value='true', description='Set to "false" to run headless.'),
        DeclareLaunchArgument('server', default_value='true', description='Set to "false" not to run gzserver.'),
        DeclareLaunchArgument('world_file', default_value=os.path.join(pkg_share, 'worlds', 'glovebox.sdf'), description='Full path to world file to load'),
        DeclareLaunchArgument('urdf_file', default_value=os.path.join(pkg_share, 'urdf', 'ElephantMyCobotPro600/mycobot_pro_600.urdf'), description='Full path to URDF file'),
        DeclareLaunchArgument('rviz_config', default_value=os.path.join(pkg_share, 'config', 'roboag.rviz'), description='Full path to RViz config file'),
        DeclareLaunchArgument('initial_joint_states', default_value='0.0 0.0 0.0 0.0 0.0 0.0', description='Initial joint states for the robot in radians')
    ]

    # Set Gazebo environment variables
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=[os.path.join(pkg_share, 'models'), ':', os.environ.get('GAZEBO_MODEL_PATH', '')]
    )

    # Gazebo Launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={
            'world': world_file,
            'gui': gui,
            'server': server,
            'use_sim_time': use_sim_time
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf_file]
    )

    # Joint State Publisher GUI Node
    joint_state_publisher_gui = Node(
        package=project_package, 
        executable='gui/joint_pos_gui',  # Assuming this is the correct path to the GUI executable
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[{'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']}]
    )

    # RViz Node
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Camera Node
    camera_node = Node(
        package=project_package,
        executable='scripts/camera',  # Adjust if this is not the correct path
        name='camera_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # AprilTag Detection Node
    apriltag_node = Node(
        package=project_package,
        executable='cv_error_detection',  # Assuming this file handles detection
        name='apriltag_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Reinforcement Learning Node
    rl_node = Node(
        package=project_package,
        executable='main',  # Assuming this is the entry point for RL
        name='reinforcement_learning_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Create and return the launch description
    return LaunchDescription(
        launch_arguments + [
            set_gazebo_model_path,
            gazebo,
            robot_state_publisher,
            joint_state_publisher_gui,  # Include the GUI node here
            rviz,
            camera_node,        # Add the camera node
            apriltag_node,     # Add the AprilTag node
            rl_node   
        ]
    )
