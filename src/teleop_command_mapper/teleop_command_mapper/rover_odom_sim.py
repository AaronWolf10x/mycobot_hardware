#!/usr/bin/env python3
"""
rover_odom_sim
==============

There's no physics engine (Gazebo or otherwise) in this workspace, so on
its own /cmd_vel does nothing visible: it's just a topic with numbers on
it. This node is what actually makes the rover drive in RViz.

Every control cycle it:
  1. Reads the most recent geometry_msgs/Twist on /cmd_vel (linear.x,
     angular.z) and integrates a simple unicycle model to update the
     rover's pose (x, y, theta) in a fixed 'odom' frame.
  2. Broadcasts the odom -> base_link TF transform for that pose. This is
     the actual thing that moves the rover in RViz -- the RobotModel and
     TF displays read TF, not /cmd_vel directly.
  3. Converts the same linear/angular velocity into left-side / right-side
     wheel angular velocities (skid-steer approximation: all left wheels
     spin together, all right wheels spin together) and integrates each
     side's spin angle so the wheel meshes visibly roll.
  4. Publishes those 4 continuous wheel joints as a sensor_msgs/JointState
     on /joint_states. robot_state_publisher merges positions from every
     publisher on that topic -- it doesn't need to be the only one. The
     arm's joints arrive as separate messages from teleop_command_mapper
     (when PUBLISH_TO_JOINT_STATES_FOR_TESTING is on there), and the two
     don't conflict because they publish disjoint sets of joint names.

This is a kinematic stand-in, not a physics simulation: no wheel slip, no
collision, no inertia, no terrain. It exists to make "pressing w/a/s/d
drives the rover the way I expect" visible before a real simulator or
real hardware is wired in.

Run it alongside rover_arm_view.launch.py:
    ros2 run teleop_command_mapper rover_odom_sim
"""

import math

import rclpy
from geometry_msgs.msg import Twist, TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

WHEEL_RADIUS = 0.08     # meters, must match cobot_rover.urdf
TRACK_WIDTH = 0.38      # meters, distance between left and right wheel centers (2 * 0.19)
UPDATE_RATE_HZ = 20.0
POSE_LOG_PERIOD_S = 0.5  # how often to print the rover's pose while moving

WHEEL_JOINT_NAMES = [
    'front_left_wheel_joint',
    'rear_left_wheel_joint',
    'front_right_wheel_joint',
    'rear_right_wheel_joint',
]


class RoverOdomSim(Node):
    def __init__(self):
        super().__init__('rover_odom_sim')

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.wheel_angle_left = 0.0
        self.wheel_angle_right = 0.0

        self.linear_x = 0.0
        self.angular_z = 0.0
        self._time_since_pose_log = 0.0

        self.tf_broadcaster = TransformBroadcaster(self)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10)

        dt = 1.0 / UPDATE_RATE_HZ
        self.timer = self.create_timer(dt, self._update)

        self.get_logger().info(
            'rover_odom_sim ready: integrating /cmd_vel into odom -> base_link TF '
            'and wheel spin. Fixed Frame in RViz should be "odom".'
        )

    def _cmd_vel_callback(self, msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def _update(self):
        dt = 1.0 / UPDATE_RATE_HZ
        v = self.linear_x
        w = self.angular_z

        # Unicycle model: integrate the rover's pose in the odom frame.
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt

        # Skid-steer wheel speeds from the same v, w (differential-drive
        # kinematics: outer/inner side speed = v +/- w * half the track width).
        v_left = v - w * (TRACK_WIDTH / 2.0)
        v_right = v + w * (TRACK_WIDTH / 2.0)
        self.wheel_angle_left += (v_left / WHEEL_RADIUS) * dt
        self.wheel_angle_right += (v_right / WHEEL_RADIUS) * dt

        self._broadcast_tf()
        self._publish_wheel_joint_states()

        if v != 0.0 or w != 0.0:
            self._time_since_pose_log += dt
            if self._time_since_pose_log >= POSE_LOG_PERIOD_S:
                self._time_since_pose_log = 0.0
                self.get_logger().info(
                    f'[DRIVE] pose (odom): x={self.x:.2f} m, y={self.y:.2f} m, '
                    f'heading={math.degrees(self.theta):.1f} deg  '
                    f'(v={v:+.2f} m/s, w={math.degrees(w):+.1f} deg/s)'
                )
        else:
            self._time_since_pose_log = 0.0

    def _broadcast_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        # Rotation about Z only, so a plain half-angle quaternion is enough
        # (no need for a full RPY-to-quaternion conversion here).
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)

    def _publish_wheel_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = WHEEL_JOINT_NAMES
        msg.position = [
            self.wheel_angle_left,   # front_left_wheel_joint
            self.wheel_angle_left,   # rear_left_wheel_joint
            self.wheel_angle_right,  # front_right_wheel_joint
            self.wheel_angle_right,  # rear_right_wheel_joint
        ]
        self.joint_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RoverOdomSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
