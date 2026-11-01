from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vocabulary'),
        DeclareLaunchArgument('settings', default_value=PathJoinSubstitution(
            [FindPackageShare('orb_slam3'), 'config', 'rgbd_settings.yaml'])),
        DeclareLaunchArgument('camera_info_topic', default_value='/benchmark/camera_info'),
        Node(package='orb_slam3', executable='ros_rgbd', name='orbslam3_rgbd', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('orb_slam3'), 'config', 'orbslam3.yaml']),
                         {'vocabulary': LaunchConfiguration('vocabulary'),
                          'settings': LaunchConfiguration('settings'),
                          'camera_info_topic': LaunchConfiguration('camera_info_topic')}]),
    ])
