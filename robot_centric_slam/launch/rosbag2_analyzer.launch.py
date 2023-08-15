from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    locomotion_analyzer = Node(
        package="robot_centric_slam",
        executable="locomotion_analyzer",
        name="locomotion_analyzer_node",
        remappings=[
            ("agv_curr_pose", "pose"),
            ("agv_map", "/map_cloud"),
            ("agv_odom", "/odometry/global"),
            ("agv_cmd_vel", "/AGV0/vox_nav/cmd_vel"),
            ("agv_imu", "/AGV0/dobbie/sensing/imu/tamagawa/imu_raw"),
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    ld = LaunchDescription()
    ld.add_action(locomotion_analyzer)

    return ld
