# simple_launch_urdf.py
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import subprocess


def _process_xacro(xacro_path):
    """Process a xacro file and return the resulting URDF string.

    Tries to use the xacro Python API first, falls back to calling the xacro CLI.
    """
    try:
        # try importing the xacro python module
        import xacro

        doc = xacro.process_file(xacro_path)
        return doc.toprettyxml(indent='  ')
    except Exception:
        # fallback to calling xacro executable
        try:
            output = subprocess.check_output(['xacro', xacro_path], stderr=subprocess.STDOUT)
            return output.decode('utf-8')
        except subprocess.CalledProcessError as e:
            raise RuntimeError(f"Failed to process xacro: {e.output.decode('utf-8')}")


def generate_launch_description():
    # 获取包的share目录路径
    pkg_share_dir = get_package_share_directory('URDF_test')

    # 首先尝试使用 xacro 文件（优先）
    xacro_file = os.path.join(pkg_share_dir, 'urdf', 'robot_car.xacro')
    urdf_file = os.path.join(pkg_share_dir, 'urdf', 'robot2.urdf')

    robot_desc = None
    if os.path.exists(xacro_file):
        try:
            robot_desc = _process_xacro(xacro_file)
        except Exception as e:
            # 记录处理 xacro 出错，但不立即失败，尝试回退到静态 URDF
            print(f"Warning: failed to process xacro '{xacro_file}': {e}")

    # 如果没有生成 robot_desc，则回退到静态 URDF 文件
    if robot_desc is None:
        if os.path.exists(urdf_file):
            with open(urdf_file, 'r') as file:
                robot_desc = file.read()
        else:
            raise FileNotFoundError(f"Neither xacro nor urdf found. Checked: {xacro_file}, {urdf_file}")

    # RViz 配置文件路径（如果存在）
    rviz_config_file = os.path.join(pkg_share_dir, 'rviz', 'test.rviz')
    rviz_arguments = []
    if os.path.exists(rviz_config_file):
        rviz_arguments = ['-d', rviz_config_file]

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=rviz_arguments
        )
    ])
