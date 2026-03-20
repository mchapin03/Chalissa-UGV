from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    xacro_file_path = PathJoinSubstitution([
        FindPackageShare('dumbot_bringup'),
        'urdf',
        'roboclaw.urdf.xacro'
    ])

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"),
                " ",
                xacro_file_path,
                " ",
                "front_port:=/dev/serial/by-id/usb-Basicmicro_Inc._USB_Roboclaw_2x15A_fron-if00",
                " ",
                "rear_port:=/dev/serial/by-id/usb-Basicmicro_Inc._USB_Roboclaw_2x15A_bac-if00",
            ]),
            value_type=str
        )
    }

    robot_controllers_path = PathJoinSubstitution([
        FindPackageShare("dumbot_bringup"),
        "params",
        "roboclaw_controllers.yaml"
    ])

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers_path],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="both",
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
        output="both",
    )

    return LaunchDescription([
        ros2_control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        robot_controller_spawner,
    ])