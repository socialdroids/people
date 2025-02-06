import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, Node
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration, FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Incluindo o launch do lfilter
        
        # Iniciando o nó leg_detector
        Node(
            package='leg_detector',
            executable='leg_detector',
            name='leg_detector',
            parameters=[{'config': FindPackageShare('leg_detector').find('leg_detector') + '/config/trained_leg_detector.yaml'}],
            remappings=[('scan', 'base_scan_filter')]
        )
    ])
