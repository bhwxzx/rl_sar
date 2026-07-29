
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    fdilink_ahrs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("fdilink_ahrs"), "launch", "ahrs_driver.launch.py")
        ),
    )

    rl_real_LW_node = Node(
        package='rl_sar',
        executable='rl_real_LW',
        name='rl_real_LW',
        output='screen',
    )

    return LaunchDescription([
        fdilink_ahrs_launch,
        rl_real_LW_node,
    ])
