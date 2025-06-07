from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with both server and client components."""
    
    container = ComposableNodeContainer(
        name='sample_components_container',
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
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='sample_client_component',
                plugin='sample_client_component::SampleClientComponent',
                name='sample_client_node',
                parameters=[
                    {'topic_name': 'sample_topic'},
                    {'client_id': 1}
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        container
    ])