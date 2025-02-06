import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Iniciando o nó leg_detector
        Node(
            package='leg_detector',
            executable='leg_detector',
            name='leg_detector',
            parameters=[{'config': FindPackageShare('leg_detector').find('leg_detector') + '/config/trained_leg_detector.yaml'}],
            remappings=[('scan', 'base_scan')],
            output='screen'
        )
    ])
