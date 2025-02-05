import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetParameter
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('people_tracker/fixed_frame', default_value='odom_combined', description='Frame used for tracking'),
        DeclareLaunchArgument('people_tracker/freq', default_value='10.0', description='Frequency of tracking'),
        DeclareLaunchArgument('people_tracker/start_distance_min', default_value='0.5', description='Minimum start distance for tracking'),
        DeclareLaunchArgument('people_tracker/reliability_threshold', default_value='0.75', description='Reliability threshold for tracking'),
        DeclareLaunchArgument('people_tracker/follow_one_person', default_value='true', description='Flag to follow one person'),

        # Kalman with velocity model covariances
        DeclareLaunchArgument('people_tracker/sys_sigma_pos_x', default_value='0.8', description='Kalman position covariance (x)'),
        DeclareLaunchArgument('people_tracker/sys_sigma_pos_y', default_value='0.8', description='Kalman position covariance (y)'),
        DeclareLaunchArgument('people_tracker/sys_sigma_pos_z', default_value='0.3', description='Kalman position covariance (z)'),
        DeclareLaunchArgument('people_tracker/sys_sigma_vel_x', default_value='0.5', description='Kalman velocity covariance (x)'),
        DeclareLaunchArgument('people_tracker/sys_sigma_vel_y', default_value='0.5', description='Kalman velocity covariance (y)'),
        DeclareLaunchArgument('people_tracker/sys_sigma_vel_z', default_value='0.5', description='Kalman velocity covariance (z)'),

        # Define the node
        Node(
            package='people_tracking_filter',
            executable='people_tracker',
            name='people_tracker',
            output='screen',
            parameters=[
                {'people_tracker/fixed_frame': 'odom_combined'},
                {'people_tracker/freq': 10.0},
                {'people_tracker/start_distance_min': 0.5},
                {'people_tracker/reliability_threshold': 0.75},
                {'people_tracker/follow_one_person': True},
                {'people_tracker/sys_sigma_pos_x': 0.8},
                {'people_tracker/sys_sigma_pos_y': 0.8},
                {'people_tracker/sys_sigma_pos_z': 0.3},
                {'people_tracker/sys_sigma_vel_x': 0.5},
                {'people_tracker/sys_sigma_vel_y': 0.5},
                {'people_tracker/sys_sigma_vel_z': 0.5},
            ]
        ),
    ])
