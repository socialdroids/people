import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    leg_detector_config = os.path.join(
        get_package_share_directory('leg_detector'),
        'config',
        'trained_leg_detector.yaml'
    )

    print(leg_detector_config)
    return LaunchDescription([
        Node(
            package='leg_detector',
            executable='leg_detector',
            name='leg_detector',
            parameters=[{'config_file': leg_detector_config}],  # Forma alternativa de carregar parâmetros
            remappings=[('scan', 'base_scan')],
            output='screen'
        )
    ])