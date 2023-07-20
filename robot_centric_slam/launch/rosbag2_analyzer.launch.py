from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ros2bag_analyzer = Node(
        package="robot_centric_slam",
        executable="ros2bag_analyzer",
        name="ros2bag_analyzer_node",
        # prefix=["xterm -e gdb -ex run --args"],
        remappings=[
            ("/agv_cloud", "/AGV0/dobbie/sensing/lidar/top/pointcloud_raw"),
            ("/agv_gnss_pose", "/AGV0/dobbie/sensing/gnss/pose"),
        ],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(ros2bag_analyzer)

    return ld
