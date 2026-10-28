import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    urdf_path = r'C:\Users\icybr\ros_2\ros2-windows\mycobot_280_pi\mycobot_280_pi.urdf'
    rviz_config = r'C:\Users\icybr\ros_2\ros2-windows\mycobot_280_pi\mycobot_view.rviz'

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
        # teleop_command_mapper is NOT started here on purpose -- it reads
        # raw keypresses, which needs its own console window with real
        # keyboard focus (same reason teleop_twist_keyboard is normally run
        # standalone rather than folded into a multi-node launch file). Run
        # it separately: `ros2 run teleop_command_mapper teleop_command_mapper`
        # in its own pixi shell, alongside this launch file.
        #
        # joint_state_publisher_gui is also intentionally NOT launched here --
        # teleop_command_mapper publishes /joint_states itself in arm-jog mode
        # (see PUBLISH_TO_JOINT_STATES_FOR_TESTING in the node), and running
        # both at once means two publishers fighting over the same topic.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])
