from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mic_monitor',
            executable='noise_monitor',   
            name='mic_monitor_node',
            output='screen',
            parameters=[{
                'device_name': 'ReSpeaker',
                'sample_rate': 16000,
                'chunk_ms': 50,
                'rms_threshold': 0.02,
                'topic_name': '/mic_monitor/noise_event',
                'publish_level': True,
                'level_topic_name': '/mic_monitor/noise_level'
            }]
        )
    ])
