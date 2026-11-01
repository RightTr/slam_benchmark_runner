from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('vocabulary'),
        DeclareLaunchArgument('settings'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/rgb/image_raw'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth_registered/image_raw'),
        DeclareLaunchArgument('imu_topic', default_value='/imu'),
        DeclareLaunchArgument('publish_tf', default_value='false'),
        DeclareLaunchArgument('imu_wait_timeout', default_value='0.5'),
        DeclareLaunchArgument('max_imu_gap', default_value='0.05'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=PathJoinSubstitution([FindPackageShare('orb_slam3'), 'config', 'orbslam3.rviz'])),
        Node(package='orb_slam3', executable='ros_rgbd_inertial', name='orbslam3_rgbd_inertial', output='screen',
             parameters=[PathJoinSubstitution([FindPackageShare('orb_slam3'), 'config', 'orbslam3.yaml']),
                         {'vocabulary': LaunchConfiguration('vocabulary'),
                          'settings': LaunchConfiguration('settings'),
                          'rgb_topic': LaunchConfiguration('rgb_topic'),
                          'depth_topic': LaunchConfiguration('depth_topic'),
                          'imu_topic': LaunchConfiguration('imu_topic'),
                          'publish_tf': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
                          'imu_wait_timeout': ParameterValue(LaunchConfiguration('imu_wait_timeout'), value_type=float),
                          'max_imu_gap': ParameterValue(LaunchConfiguration('max_imu_gap'), value_type=float)}]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             arguments=['-d', LaunchConfiguration('rviz_config')], condition=IfCondition(LaunchConfiguration('start_rviz')),
             output='screen'),
    ])
