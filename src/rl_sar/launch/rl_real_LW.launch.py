
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    enable_debug_publisher = LaunchConfiguration('enable_debug_publisher')
    enable_keyboard = LaunchConfiguration('enable_keyboard')

    fdilink_ahrs_launch = GroupAction(
        actions=[
            SetRemap(src='/imu', dst='/fdilink/raw_imu'),
            SetRemap(src='/euler_angles', dst='/fdilink/raw_euler'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("fdilink_ahrs"), "launch", "ahrs_driver.launch.py")
                ),
            ),
        ],
    )

    rl_real_LW_node = Node(
        package='rl_sar',
        executable='rl_real_LW',
        name='rl_real_LW',
        output='screen',
        parameters=[{
            'enable_debug_publisher': enable_debug_publisher,
            'enable_keyboard': enable_keyboard,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_keyboard',
            default_value='true',
            description='Enable real-robot keyboard input from the controlling terminal',
        ),
        DeclareLaunchArgument(
            'enable_debug_publisher',
            default_value='false',
            description='Enable the 250 Hz /LW_joint_states debug publisher',
        ),
        fdilink_ahrs_launch,
        rl_real_LW_node,
    ])
