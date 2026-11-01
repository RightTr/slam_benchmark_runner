from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [DeclareLaunchArgument('vocabulary'), DeclareLaunchArgument('settings')]
    params = {'vocabulary': LaunchConfiguration('vocabulary'), 'settings': LaunchConfiguration('settings')}
    args.append(DeclareLaunchArgument('do_rectify', default_value='false'))
    params['do_rectify'] = ParameterValue(LaunchConfiguration('do_rectify'), value_type=bool)
    args.extend([DeclareLaunchArgument('left_topic', default_value='/camera/left/image_raw'),
                 DeclareLaunchArgument('right_topic', default_value='/camera/right/image_raw')])
    params['left_topic'] = LaunchConfiguration('left_topic')
    params['right_topic'] = LaunchConfiguration('right_topic')
    args.append(DeclareLaunchArgument('imu_topic', default_value='/imu'))
    params['imu_topic'] = LaunchConfiguration('imu_topic')
    args.append(DeclareLaunchArgument('do_equalize', default_value='false'))
    params['do_equalize'] = ParameterValue(LaunchConfiguration('do_equalize'), value_type=bool)
    args.append(Node(package='orb_slam3', executable='ros_stereo_inertial', name='orbslam3_stereo_inertial', output='screen', parameters=[params]))
    return LaunchDescription(args)
