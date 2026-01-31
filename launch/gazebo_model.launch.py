import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro


def generate_launch_description():
    robot_xacro_name = "differential_drive_robot"
    package_name = "mobile_robot"
    model_file_relative_path = "model/robot.xacro"

    path_model_file = os.path.join(
        get_package_share_directory(package_name), model_file_relative_path
    )

    robot_description = xacro.process_file(path_model_file).toxml()

    gazebo_launch_source = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
        )
    )

    gazebo_launch = IncludeLaunchDescription(
        gazebo_launch_source,
        launch_arguments={
            "gz_args": "-r -v -v4 empty.sdf",
            "on_exit_shutdown": "true",
        }.items(),
    )

    spawn_model_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", robot_xacro_name, "-topic", "robot_description"],
        output="screen",
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    bridge_params = os.path.join(
        get_package_share_directory(package_name),
        "parameters",
        "bridge_parameters.yaml",
    )

    gazebo_ros_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
        output="screen",
    )

    launch_description = LaunchDescription()

    launch_description.add_action(gazebo_launch)
    launch_description.add_action(spawn_model_node)
    launch_description.add_action(robot_state_publisher_node)
    launch_description.add_action(gazebo_ros_bridge_node)

    return launch_description
