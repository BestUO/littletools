"""
Launch Gazebo, run robot_state_publisher with the URDF, and spawn the mycar model.

This launch file does the following:
- start Gazebo (using ros_gz_sim)
- publish /robot_description via robot_state_publisher
- spawn the URDF into Gazebo using ros_gz_sim

Usage: ros2 launch Gazebo_test mycar.launch.py
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, FindExecutable, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare('Gazebo_test').find('Gazebo_test')

    # paths
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'mycar.urdf'])
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'robot_car.xacro'])

    # Start Gazebo using ros_gz_sim
    # If a world file exists under this package at `worlds/world.sdf`, use it; otherwise use empty.sdf
    candidate_world = os.path.join(pkg_share, 'worlds', 'world.sdf')
    if os.path.exists(candidate_world):
        gz_args_value = f"{candidate_world} -v 4"
    else:
        gz_args_value = 'empty.sdf -v 4'  # 空世界，verbose level 4

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ]),
        launch_arguments={
            'gz_args': gz_args_value
        }.items()
    )

    # Robot description: prefer processing the XACRO (robot_car.xacro) via xacro
    # If you want to keep using a pre-generated URDF, change this to use `urdf_file`.
    robot_desc = Command([
        FindExecutable(name='xacro'), ' ', xacro_file
    ])

    # robot_state_publisher to publish TFs
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': True  # 重要：使用仿真时间
        }]
    )

    # Spawn entity using ros_gz bridge (新的方式)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mycar',
            '-x', '0.0',
            '-y', '0.0', 
            '-z', '0.1'
        ],
        output='screen'
    )

    # 可选的：启动 ros_gz_bridge 来转发话题（如果需要 ROS 2 与 Gazebo 通信）
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            # 可以根据需要添加更多桥接话题
            # '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            # '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry'
        ],
        output='screen'
    )

    ld = LaunchDescription()
    
    # 首先启动 Gazebo
    ld.add_action(gazebo_launch)
    
    # 然后启动 bridge（可选）
    ld.add_action(bridge_node)
    
    # 启动 robot_state_publisher
    ld.add_action(rsp_node)
    
    # 延迟一段时间后生成机器人（确保 Gazebo 已启动）
    ld.add_action(TimerAction(
        period=3.0,  # 延迟 3 秒
        actions=[spawn_entity]
    ))

    return ld