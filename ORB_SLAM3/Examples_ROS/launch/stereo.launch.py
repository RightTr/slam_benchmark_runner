from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [DeclareLaunchArgument('vocabulary'), DeclareLaunchArgument('settings')]
    params = {'vocabulary': LaunchConfiguration('vocabulary'), 'settings': LaunchConfiguration('settings')}
    args.extend([DeclareLaunchArgument('left_topic', default_value='/camera/left/image_raw'),
                 DeclareLaunchArgument('right_topic', default_value='/camera/right/image_raw')])
    params['left_topic'] = LaunchConfiguration('left_topic')
    params['right_topic'] = LaunchConfiguration('right_topic')
    args.append(DeclareLaunchArgument('do_rectify', default_value='false'))
    params['do_rectify'] = ParameterValue(LaunchConfiguration('do_rectify'), value_type=bool)
    args.append(Node(package='orb_slam3', executable='ros_stereo', name='orbslam3_stereo', output='screen', parameters=[params]))
    return LaunchDescription(args)
