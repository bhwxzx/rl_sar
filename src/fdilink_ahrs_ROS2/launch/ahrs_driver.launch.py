from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ahrs_driver = Node(
        package='fdilink_ahrs',
        executable='ahrs_driver_node',
        parameters=[{
            'if_debug_': False,
            'serial_port_': '/dev/fdilink_ahrs',
            'serial_baud_': 921600,
            'imu_topic': '/imu',
            'imu_frame_id_': 'gyro_link',
            'mag_pose_2d_topic': '/mag_pose_2d',
            'Magnetic_topic': '/magnetic',
            'Euler_angles_topic': '/euler_angles',
            'gps_topic': '/gps/fix',
            'twist_topic': '/system_speed',
            'NED_odom_topic': '/NED_odometry',
            'device_type_': 1,
        }],
        output='screen',
    )

    launch_description = LaunchDescription()
    launch_description.add_action(ahrs_driver)
    return launch_description
