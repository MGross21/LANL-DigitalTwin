import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    TimerAction,
    ExecuteProcess,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directory
    pkg_share = get_package_share_directory('RoboAgRL')

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
            'verbose': 'true'
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command(['xacro ', urdf_file])},
            {'use_sim_time': use_sim_time},
        ]
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time, 'initial_joint_states': initial_joint_states}]
    )

    # Spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'ElephantMyCobotPro600', 
                   '-topic', 'robot_description',
                   '-x', '0', '-y', '0', '-z', '1.5'],
        output='screen'
    )

    # RViz Visualization
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # TF Echo for Debugging
    tf_echo = Node(
        package='tf2_ros',
        executable='tf2_echo',
        name='tf2_echo',
        arguments=['base', 'tool'],
        output='screen'
    )

    # Check URDF Process
    check_urdf = ExecuteProcess(
        cmd=['check_urdf', urdf_file],
        output='screen'
    )

    # Return the Launch Description
    return LaunchDescription(
        launch_arguments + [
            set_gazebo_model_path,
            check_urdf,
            gazebo,
            robot_state_publisher,
            joint_state_publisher,
            TimerAction(period=10.0, actions=[spawn_entity]),
            TimerAction(period=15.0, actions=[rviz, tf_echo])
        ]
    )
