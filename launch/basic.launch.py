import os
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import xacro

PACKAGE_NAME = "camera_calibration_gazebo"
WORLD = "calibrate.world"
MODEL = "depth_camera"

def generate_launch_description():
    pkg_share = get_package_share_directory(PACKAGE_NAME)
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    model_sdf_full_path = os.path.join(pkg_share, "models", MODEL, "model.sdf")

    
    resources = [
        os.path.join(pkg_share, "worlds")    
    ]


    models = [
        os.path.join(pkg_share, "models"),
        "/home/user/ros2_ws/src/camera_calibration_gazebo/model"

    ]

    sdf_models_path = "/home/user/ros2_ws/src/camera_calibration_gazebo/model/landmark/landmark.sdf.xacro"
    robot_description_config = xacro.process_file(sdf_models_path)
    robot_description =  robot_description_config.toxml()

    resource_env = AppendEnvironmentVariable(name="GAZEBO_RESOURCE_PATH", value=":".join(resources))
    models_env = AppendEnvironmentVariable(name="GAZEBO_MODEL_PATH", value=":".join(models))

    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            "verbose": "true", 
            'world': WORLD}.items())

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')))
        
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {"use_sim_time": True},
            {"robot_description": robot_description},
        ],
    )

    spawn_entity_cmd = Node(
        package="gazebo_ros", 
        executable="spawn_entity.py",
        arguments=['-entity', "landmark", 
        '-topic', "robot_description",
        '-x', "1",
        '-y', "0",
        '-z', "2",
        "-P", "1.575"],
        output='screen')



    ld = LaunchDescription()
    ld.add_action(resource_env)
    ld.add_action(models_env)
    
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_entity_cmd)

    return ld