from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    ld = LaunchDescription()
    node_args = {
        "package": "realsense_post_processing",
        "executable": "realsense_post_processing",
        "output": "screen",
    }
    ros_distro = os.environ.get("ROS_DISTRO", "humble")

    # config_file = os.path.join(
    #     get_package_share_directory("realsense_post_processing"),
    #     "params",
    #     "params.yaml",
    # )

    talker_node = Node(**node_args) #, parameters=[config_file])

    ld.add_action(talker_node)
    return ld
