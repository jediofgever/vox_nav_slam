from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    imu_locomotion = Node(
        package="robot_centric_slam",
        executable="imu_locomotion",
        name="imu_locomotion_node",
        remappings=[
            # ("agv_map", "/map_cloud"),
            ("agv_map", "/lio_sam/mapping/map_local"),
            ("agv_imu", "/imu/data/corrected"),
        ],
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "look_back_poses": 25,
                "dist_threshold": 0.2,
            },
        ],
        # prefix=["xterm -e gdb -ex run --args"],
    )

    ld = LaunchDescription()
    ld.add_action(imu_locomotion)

    return ld
