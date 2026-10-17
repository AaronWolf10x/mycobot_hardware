import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    driver = LaunchConfiguration("driver").perform(context)
    port = LaunchConfiguration("port").perform(context)

    desc_dir = get_package_share_directory("mycobot_description")
    hw_dir = get_package_share_directory("mycobot_hardware")
    controllers_yaml = os.path.join(hw_dir, "config", "controllers.yaml")

    with open(os.path.join(desc_dir, "urdf", "mycobot_280_pi", "mycobot_280_pi.urdf")) as f:
        urdf = f.read()
    # Vendor URDF sets every joint velocity limit to 0, which freezes the joints.
    urdf = re.sub(r'velocity\s*=\s*"0(?:\.0*)?"', 'velocity="1.5"', urdf)
    with open(os.path.join(hw_dir, "urdf", "ros2_control_block.xml")) as f:
        block = f.read()

    block = block.replace(">mock<", f">{driver}<").replace("/dev/ttyUSB0", port)
    head, sep, tail = urdf.rpartition("</robot>")
    robot_description = head + block + sep + tail

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )
    cm = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_yaml],
        remappings=[("~/robot_description", "/robot_description")],
        output="screen",
    )
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--param-file", controllers_yaml,
        ],
    )
    return [rsp, cm, jsb_spawner, arm_spawner]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("driver", default_value="mock"),
        DeclareLaunchArgument("port", default_value="/dev/ttyUSB0"),
        OpaqueFunction(function=launch_setup),
    ])
