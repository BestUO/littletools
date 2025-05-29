from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'client1_count',
            default_value='5',
            description='Message count for client 1'
        ),
        DeclareLaunchArgument(
            'client2_count', 
            default_value='8',
            description='Message count for client 2'
        ),
        Node(
            package='sample_client',
            executable='SampleClient',
            name='sample_client_node_1',
            namespace='sample_name_space',
            parameters=[{
                'message_count': LaunchConfiguration('client1_count'),
                'client_id': 1,
                'action_name': 'custom_action_1',
            }],
            remappings=[
                ('sample_topic', 'custom_topic_1'),
                ('sample_service', 'custom_service_1'),
            ],
            output='screen'
        ),
        Node(
            package='sample_client',
            executable='SampleClient', 
            name='sample_client_node_2',
            namespace='sample_name_space',
            parameters=[{
                'message_count': LaunchConfiguration('client2_count'),
                'client_id': 2,
                'action_name': 'custom_action_2',
            }],
            remappings=[
                ('sample_topic', 'custom_topic_2'),
                ('sample_service', 'custom_service_2'),
            ],
            output='screen'
        )
    ])