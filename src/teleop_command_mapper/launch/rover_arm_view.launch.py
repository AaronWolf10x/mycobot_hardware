import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    urdf_path = r'C:\Users\icybr\ros_2\ros2-windows\mycobot_280_pi\cobot_rover.urdf'
    rviz_config = r'C:\Users\icybr\ros_2\ros2-windows\mycobot_280_pi\rover_view.rviz'

    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    os.environ['QT_ENABLE_HIGHDPI_SCALING'] = '0'
    os.environ['QT_SCALE_FACTOR'] = '1'
    os.environ['QT_AUTO_SCREEN_SCALE_FACTOR'] = '0'
    os.environ['QT_SCALE_FACTOR_ROUNDING_POLICY'] = 'PassThrough'

    return LaunchDescription([
        SetEnvironmentVariable('QT_ENABLE_HIGHDPI_SCALING', '0'),
        SetEnvironmentVariable('QT_SCALE_FACTOR', '1'),
        SetEnvironmentVariable('QT_AUTO_SCREEN_SCALE_FACTOR', '0'),
        SetEnvironmentVariable('QT_SCALE_FACTOR_ROUNDING_POLICY', 'PassThrough'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        # rover_odom_sim runs here (not in its own window) because it doesn't
        # read the keyboard -- it just turns /cmd_vel into TF + wheel motion,
        # so it can share this launch file with robot_state_publisher/rviz2.
        Node(
            package='teleop_command_mapper',
            executable='rover_odom_sim',
            name='rover_odom_sim',
            output='screen',
        ),
        # teleop_command_mapper is NOT started here on purpose -- same reason
        # as in teleop_view.launch.py: it reads raw keypresses and needs its
        # own console window with real keyboard focus. Run it separately:
        #   ros2 run teleop_command_mapper teleop_command_mapper
        # in its own pixi shell, alongside this launch file.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
