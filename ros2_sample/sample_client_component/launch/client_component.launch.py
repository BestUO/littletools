from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with a component container."""
    container = ComposableNodeContainer(
        name='sample_client_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='sample_client_component',
                plugin='sample_client_component::SampleClientComponent',
                name='sample_client_node',
                parameters=[
                    {'topic_name': 'sample_topic'},
                    {'client_id': 1}
                ]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])