from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with cross-communication pattern."""
    container = ComposableNodeContainer(
        name='sample_components_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Server 1 - Fast publisher
            ComposableNode(
                package='sample_server_component',
                plugin='sample_server_component::SampleServerComponent',
                name='fast_server',
                parameters=[
                    {'publish_frequency': 3.0},
                    {'topic_name': 'fast_topic'}
                ],
                remappings=[
                    ('sample_topic', 'fast_topic')
                ]
            ),
            # Server 2 - Slow publisher  
            ComposableNode(
                package='sample_server_component',
                plugin='sample_server_component::SampleServerComponent',
                name='slow_server',
                parameters=[
                    {'publish_frequency': 0.5},
                    {'topic_name': 'slow_topic'}
                ],
                remappings=[
                    ('sample_topic', 'slow_topic')
                ]
            ),
            # Client 1 - Monitor fast server
            ComposableNode(
                package='sample_client_component',
                plugin='sample_client_component::SampleClientComponent',
                name='fast_monitor',
                parameters=[
                    {'topic_name': 'fast_topic'},
                    {'client_id': 101}
                ],
                remappings=[
                    ('sample_topic', 'fast_topic')
                ]
            ),
            # Client 2 - Monitor slow server
            ComposableNode(
                package='sample_client_component',
                plugin='sample_client_component::SampleClientComponent',
                name='slow_monitor',
                parameters=[
                    {'topic_name': 'slow_topic'},
                    {'client_id': 102}
                ],
                remappings=[
                    ('sample_topic', 'slow_topic')
                ]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])