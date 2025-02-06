import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import FindPackageShare, LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Iniciando o nó laser_filter com o scan_to_scan_filter_chain
        Node(
            package='laser_filters',
            executable='scan_to_scan_filter_chain',
            name='laser_filter',
            respawn=True,
            parameters=[{'rosparam': FindPackageShare('map_laser').find('map_laser') + '/config/filters.yaml'}],
            remappings=[('scan', 'base_scan')]
        ),
        
        # Iniciando o nó leg_detector
        Node(
            package='leg_detector',
            executable='leg_detector',
            name='leg_detector',
            parameters=[{'config': FindPackageShare('leg_detector').find('leg_detector') + '/config/trained_leg_detector.yaml'}],
            remappings=[('scan', 'scan_filtered')]
        )
    ])
