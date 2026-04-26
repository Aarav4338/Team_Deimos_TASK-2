import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Locate the ekf.yaml config file inside the installed package
    pkg_share = get_package_share_directory('mybot_localization')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    return LaunchDescription([

        # 1. EKF Node (robot_localization)
        # Fuses odom0 (wheel odometry) and imu0 (IMU) into a single
        # smooth /odometry/filtered output.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config],
        ),

        # 2. Static Transform Publisher (The Catch)
        #
        # The IMU is physically mounted BACKWARDS on the rover,
        # rotated 180 degrees around the Z-axis.
        #
        # We publish a static transform from base_link -> imu_link
        # with yaw = 3.14159 (pi radians = 180 degrees).
        # This tells the EKF to mathematically flip the IMU data
        # before fusing it, preventing filter divergence.
        #
        # Arguments (legacy format): x y z yaw pitch roll frame_id child_frame_id
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_static_tf',
            output='screen',
            arguments=['0', '0', '0', '3.14159', '0', '0', 'base_link', 'imu_link'],
        ),

    ])
