from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory("board_pose_ros")
    config_dir = os.path.join(pkg_share, "config")

    calib_file = os.path.join(config_dir, "camera_calib_oak.npz")
    board_file = os.path.join(config_dir, "board_config.json")

    return LaunchDescription([
        Node(
            package="board_pose_ros",
            executable="board_pose_node",
            name="board_pose_node",
            output="screen",
            parameters=[{
                "calib": calib_file,
                "config": board_file,
                "width": 1280,
                "height": 720,
                "alpha": 0.25,
                "camera_frame": "oak_camera_frame",
                "board_frame": "board_frame",
                "fps": 30.0,
            }],
        )
    ])