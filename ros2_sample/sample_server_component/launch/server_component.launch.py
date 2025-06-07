from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with a component container."""
    container = ComposableNodeContainer(
        name='sample_server_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='sample_server_component',
                plugin='sample_server_component::SampleServerComponent',
                name='sample_server_node',
                parameters=[
                    {'publish_frequency': 2.0},
                    {'topic_name': 'sample_topic'}
                ]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])