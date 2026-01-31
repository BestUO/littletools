from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        SetEnvironmentVariable(
            name='ASAN_OPTIONS',
            value='new_delete_type_mismatch=0:log_path=/home/robot/resource/log/asan.log',
        ),
        Node(
            package='files_upload_mgr',
            executable='files_upload_mgr',
            respawn=False,
            output='screen',
            parameters=[{
            }]
        ),
    ])
