#!/usr/bin/env python3
"""
arm_command_bridge
===================

This week's task: connect our nodes together. Both teleop_command_mapper_node.py
(keyboard) and joy_command_mapper_node.py (gamepad) already publish the full
6-joint target as a sensor_msgs/msg/JointState on /arm_joint_command every time
it changes (see _publish_joint_targets in either node).

Aaron's mycobot_hardware package is a ros2_control hardware interface
(MyCobotSystem, in mycobot_system.cpp) -- it does NOT subscribe to any topic
itself; ros2_control components never do. Commands only reach it through a
*controller*. Aaron wired up arm_controller, a
joint_trajectory_controller/JointTrajectoryController (see
mycobot_hardware/config/controllers.yaml), which accepts a
trajectory_msgs/msg/JointTrajectory on /arm_controller/joint_trajectory and
writes each point into the position command interfaces that
MyCobotSystem::write() reads via get_command(name + "/position").

This node is the adapter between those two message types: every JointState
that arrives on /arm_joint_command becomes one single-point JointTrajectory,
republished immediately on /arm_controller/joint_trajectory. No
interpolation math needed here -- JointTrajectoryController already handles
a steady stream of short, single-point trajectories like this fine; that's
the normal way to drive it for teleop-style "go to the latest target now"
control, as opposed to a pre-planned multi-waypoint trajectory.

(Note: this is plain inter-process pub/sub over a topic, same as any other
two ROS2 nodes -- that's all "connecting nodes" requires. Intra-process
communication, https://docs.ros.org/en/lyrical/.../Intra-Process-Communication.html,
is an unrelated, optional *optimization* for when multiple nodes are loaded
as components into the same process to skip serialization -- it's not a
prerequisite for nodes to talk to each other, and at arm_controller's 25 Hz
update_rate over a request/response serial link, there's nothing here that
calls for it.)

Run alongside whichever mapper node you're using, plus mycobot_hardware's
bringup launch file:
    ros2 launch mycobot_hardware bringup.launch.py driver:=mock
    ros2 run teleop_command_mapper teleop_command_mapper   # or joy_command_mapper
    ros2 run teleop_command_mapper arm_command_bridge
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

SOURCE_TOPIC = '/arm_joint_command'
TARGET_TOPIC = '/arm_controller/joint_trajectory'

# How far into the future each trajectory point's target is set. This is
# deliberately short -- a single-point trajectory like this isn't a planned
# move, it's "get to the latest target soon". Long enough for arm_controller
# (25 Hz update_rate, see controllers.yaml) to interpolate smoothly between
# back-to-back commands; short enough that jogging/the scoop action still
# feels responsive rather than laggy.
TIME_FROM_START_S = 0.15


class ArmCommandBridge(Node):
    def __init__(self):
        super().__init__('arm_command_bridge')
        self.pub = self.create_publisher(JointTrajectory, TARGET_TOPIC, 10)
        self.sub = self.create_subscription(
            JointState, SOURCE_TOPIC, self._on_joint_command, 10)
        self.get_logger().info(
            f'arm_command_bridge: relaying {SOURCE_TOPIC} (JointState) -> '
            f'{TARGET_TOPIC} (JointTrajectory) for mycobot_hardware\'s '
            f'arm_controller.'
        )

    def _on_joint_command(self, msg: JointState):
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = list(msg.name)

        point = JointTrajectoryPoint()
        point.positions = list(msg.position)
        whole_s = int(TIME_FROM_START_S)
        point.time_from_start = Duration(
            sec=whole_s, nanosec=int((TIME_FROM_START_S - whole_s) * 1e9))
        traj.points = [point]

        self.pub.publish(traj)


def main(args=None):
    rclpy.init(args=args)
    node = ArmCommandBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
