from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    timne_window = DeclareLaunchArgument(
        'timne_window',
        default_value='1.0',
        description='timne_window'
    )
    topicname = DeclareLaunchArgument(
        'topicname',
        default_value='/ecu/currents/values',
        description='topic name of currents'
    )

    node = Node(
        package='current_subscriber',
        executable='current_subscriber',
        name='current_subscriber',
        output='screen',
        parameters=[{
            'topicname': LaunchConfiguration('topicname'),
            'timne_window': LaunchConfiguration('timne_window'),
        }]
    )

    return LaunchDescription([
        timne_window,
        topicname,
        node
    ])
