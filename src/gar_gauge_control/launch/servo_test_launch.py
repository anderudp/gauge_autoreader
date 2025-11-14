from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    servo_config = os.path.join(
        get_package_share_directory('gar_gauge_control'),
        'config',
        'servo',
        'XC330-T181.yaml'
    )

    return LaunchDescription([
        Node(
            package='gar_gauge_control',
            executable='read_write_node',
            name='read_write_node',
            parameters=[
                servo_config
            ]
        ),
        Node(
            package='gar_gauge_control',
            executable='servo_tester_node',
            name='servo_tester_node'
        ),
    ])