# ros2 run camera_calibration cameracalibrator --size 7x9 --square 0.03 --ros-args -r image:=/raw_camera/image_raw -p camera:=/raw_camera
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():

    node = Node(
        package="turtlesim",
        executable="turtlesim_node",
    )
    ld = LaunchDescription()
    ld.add_action(node)

    return ld