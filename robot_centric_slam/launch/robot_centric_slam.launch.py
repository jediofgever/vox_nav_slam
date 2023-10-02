from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    share_dir = get_package_share_directory("robot_centric_slam")

    xacro_path = os.path.join(share_dir, "param", "robot_sim.urdf.xacro")

    params = LaunchConfiguration("params")

    decleare_params = DeclareLaunchArgument(
        "params",
        default_value=os.path.join(share_dir, "param", "vox_nav_slam_node.yaml"),
        description="Path to the vox_nav parameters file.",
    )

    mapping = Node(
        package="robot_centric_slam",
        executable="vox_nav_slam_node",
        name="vox_nav_slam_node",
        prefix=["xterm -e gdb -ex run --args"],
        parameters=[params],
        remappings=[
            ("/cloud_in", "/AGV0/dobbie/sensing/lidar/top/pointcloud_raw_ex"),
            # ("/odom_in", "/odometry/global"),
            ("/odom_in", "/kiss_icp/odometry"),
        ],
        output="screen",
    )

    tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments="0.0 0.0 0.0 0.0 0.0 0.0 map odom".split(" "),
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
            }
        ],
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": Command(["xacro", " ", xacro_path]),
                "use_sim_time": True,
            }
        ],
    )

    ld = LaunchDescription()
    ld.add_action(decleare_params)
    ld.add_action(mapping)
    ld.add_action(rsp)

    return ld
