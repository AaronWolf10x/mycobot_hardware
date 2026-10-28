#!/usr/bin/env python3
"""
joy_command_mapper
====================

Gamepad/on-screen-joystick counterpart to teleop_command_mapper_node.py.
Same two modes, same outputs, different input device:

  DRIVE mode (default)   -> publishes geometry_msgs/Twist on /cmd_vel
  ARM_JOG mode            -> continuously jogs the end-effector in
                             Cartesian space and publishes the resulting
                             joint angles as a sensor_msgs/JointState on
                             /arm_joint_command

This node doesn't read a gamepad directly -- it subscribes to the
standard ROS 2 sensor_msgs/msg/Joy topic, /joy, so it works unchanged no
matter what actually publishes that topic:
  - a real USB gamepad through the ROS 2 `joy` package's `joy_node`
  - the on-screen simulator in this same package, joy_sim_gui.py, if you
    don't have a physical gamepad yet
  - a virtual joystick phone app bridged to /joy over UDP

Axis/button layout (ROS 2 joy convention: left = positive on X axes, up =
positive on Y axes). These are parameters (ros2 run ... --ros-args -p
name:=value) so you can remap for whatever controller you actually have
without editing code:
    drive_linear_axis   (default 1): left stick vertical    -> linear.x
    drive_angular_axis  (default 0): left stick horizontal  -> angular.z
    jog_x_axis          (default 1): left stick vertical    -> end-effector X
    jog_y_axis          (default 0): left stick horizontal  -> end-effector Y
    jog_z_axis          (default 4): right stick vertical   -> end-effector Z
    mode_button         (default 0): press to toggle DRIVE <-> ARM_JOG
    scoop_button        (default 1): press to run the excavator-style
                                      "scoop action" -- sweep the boom/
                                      stick/bucket joints down into a dig,
                                      pause, sweep back up (works in
                                      either mode; Neil's note, Oct
                                      23rd/25th meeting)

Deliberately reuses the same two stick axes for both modes, the same way
the keyboard node reuses w/x/a/d for both drive and jog -- whichever mode
you're in decides what the stick does, not which stick you touch.

The Cartesian jogging uses the exact same differential-IK machinery
(Jacobian pseudo-inverse over the hand-built KDL chain) as the keyboard
node -- see arm_jog_kinematics.py, shared by both nodes so the two input
methods can't drift out of sync with each other. The scoop action is
shared the same way (ScoopSequencer, also in arm_jog_kinematics.py), so
the gamepad's scoop button and the keyboard's 'g' key do exactly the same
motion -- but it does NOT go through that same Jacobian jogging. A scoop
needs a few joints each making one big, excavator-like sweep, and the
Jacobian pseudo-inverse's job is the opposite of that: it always finds
the *smallest* total joint movement for a given Cartesian step, which
for this 6-joint/3-DOF arm means spreading the motion thin across every
joint instead of sweeping a few clearly. So the scoop drives
BOOM_JOINT/STICK_JOINT/BUCKET_JOINT directly instead (see
arm_jog_kinematics.py).
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy, JointState

from .arm_jog_kinematics import JOINT_NAMES, ArmJogger, ScoopSequencer

DRIVE = 'drive'
ARM_JOG = 'arm_jog'

CONTROL_RATE_HZ = 20.0
LINEAR_SPEED = 0.5            # m/s at full stick deflection, drive mode
ANGULAR_SPEED = 1.0           # rad/s at full stick deflection, drive mode
CARTESIAN_SPEED = 0.08        # m/s at full stick deflection, arm-jog mode
DEADZONE = 0.15               # ignore stick noise below this magnitude

# Same testing aid as the keyboard node: also publish the jogged joint
# angles on /joint_states so you can see the arm move in RViz without a
# real hardware relay node yet. Turn this OFF once that relay exists, so
# the two don't fight over the same topic. If you're running this
# alongside teleop_command_mapper (the keyboard node) at the same time,
# turn it off on at least one of them -- they'd otherwise both think
# they own /joint_states.
PUBLISH_TO_JOINT_STATES_FOR_TESTING = True


def apply_deadzone(value, deadzone=DEADZONE):
    return 0.0 if abs(value) < deadzone else value


class JoyCommandMapper(Node):
    def __init__(self):
        super().__init__('joy_command_mapper')

        self.declare_parameter('drive_linear_axis', 1)
        self.declare_parameter('drive_angular_axis', 0)
        self.declare_parameter('jog_x_axis', 1)
        self.declare_parameter('jog_y_axis', 0)
        self.declare_parameter('jog_z_axis', 4)
        self.declare_parameter('mode_button', 0)
        self.declare_parameter('scoop_button', 1)

        self.drive_linear_axis = self.get_parameter('drive_linear_axis').value
        self.drive_angular_axis = self.get_parameter('drive_angular_axis').value
        self.jog_x_axis = self.get_parameter('jog_x_axis').value
        self.jog_y_axis = self.get_parameter('jog_y_axis').value
        self.jog_z_axis = self.get_parameter('jog_z_axis').value
        self.mode_button = self.get_parameter('mode_button').value
        self.scoop_button = self.get_parameter('scoop_button').value

        self.mode = DRIVE
        self.arm = ArmJogger()
        self.scoop = ScoopSequencer(self.arm.apply_joint_deltas, on_state_change=self._on_scoop_state_change)
        self.last_joy = None
        self._mode_button_was_down = False
        self._scoop_button_was_down = False
        self._time_since_jog_log = 0.0

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_command_pub = self.create_publisher(JointState, '/arm_joint_command', 10)
        if PUBLISH_TO_JOINT_STATES_FOR_TESTING:
            self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
            self._publish_joint_targets()  # resting pose visible immediately, same reasoning
            # as teleop_command_mapper_node.py's startup publish: don't wait on a jog to
            # give robot_state_publisher a value for these joints.
            #
            # A single startup publish isn't enough on its own, though: if
            # /joy never arrives (no joy_sim_gui/joy_node running) or the
            # sticks just sit centered, _do_arm_jog() returns early and
            # never republishes -- so this one message might be the only
            # one ever sent for joint2..joint6_flange. If it goes out
            # before robot_state_publisher has finished discovering this
            # publisher (DDS discovery takes a moment), it's silently
            # dropped and RViz is left showing "No transform from
            # [jointN]" forever. A periodic re-publish (same fix already
            # applied to teleop_command_mapper_node.py) makes that
            # impossible -- joint state keeps arriving every 0.5s no
            # matter what /joy is doing.
            self.joint_state_timer = self.create_timer(0.5, self._publish_joint_targets)

        self.joy_sub = self.create_subscription(Joy, '/joy', self._joy_callback, 10)
        self.control_timer = self.create_timer(1.0 / CONTROL_RATE_HZ, self._control_step)

        self.get_logger().info(
            'joy_command_mapper ready, currently: DRIVE. Waiting for /joy '
            '(a real gamepad via the joy package, or joy_sim_gui.py if you '
            "don't have one yet). Press the mode button to switch to ARM_JOG, "
            'or the scoop button (either mode) to run the excavator-style dig.'
        )

    def _joy_callback(self, msg):
        self.last_joy = msg

    def _axis(self, msg, index):
        if index is None or index < 0 or index >= len(msg.axes):
            return 0.0
        return apply_deadzone(msg.axes[index])

    def _button_down(self, msg, index):
        if index is None or index < 0 or index >= len(msg.buttons):
            return False
        return bool(msg.buttons[index])

    def _control_step(self):
        msg = self.last_joy
        if msg is None:
            return  # no /joy data yet -- nothing to do

        # Edge-detect the mode button so a held-down press doesn't toggle
        # repeatedly (same intent as the keyboard node's single TAB keypress).
        button_down = self._button_down(msg, self.mode_button)
        if button_down and not self._mode_button_was_down:
            self.mode = ARM_JOG if self.mode == DRIVE else DRIVE
            self.get_logger().info(f'Mode switched to: {self.mode.upper()}')
            if self.mode == DRIVE:
                self.cmd_vel_pub.publish(Twist())  # make sure the rover stops
        self._mode_button_was_down = button_down

        # Scoop button: same edge-detect pattern as the mode button, but
        # not gated by self.mode -- it only moves the arm (never
        # /cmd_vel), so it works the same whether you're currently in
        # DRIVE or ARM_JOG.
        scoop_button_down = self._button_down(msg, self.scoop_button)
        if scoop_button_down and not self._scoop_button_was_down:
            if not self.scoop.start():
                self.get_logger().info('Scoop already in progress -- ignoring button press.')
        self._scoop_button_was_down = scoop_button_down

        if self.scoop.is_active():
            self.scoop.tick(1.0 / CONTROL_RATE_HZ)
            self._publish_joint_targets()

        if self.mode == DRIVE:
            self._do_drive(msg)
        elif not self.scoop.is_active():
            # Scoop action owns the arm while it's running -- skip manual
            # stick jogging so the two don't fight over joint_positions.
            self._do_arm_jog(msg)

    def _on_scoop_state_change(self, old_state, new_state):
        messages = {
            ScoopSequencer.DESCENDING: 'Scoop action: sweeping boom/stick/bucket joints down into the dig...',
            ScoopSequencer.DWELLING: 'Scoop action: material captured, pausing briefly...',
            ScoopSequencer.ASCENDING: 'Scoop action: sweeping boom/stick/bucket joints back up...',
            ScoopSequencer.IDLE: 'Scoop action: complete.',
        }
        message = messages.get(new_state)
        if message:
            self.get_logger().info(message)

    # -- mode: DRIVE ----------------------------------------------------
    def _do_drive(self, msg):
        twist = Twist()
        twist.linear.x = self._axis(msg, self.drive_linear_axis) * LINEAR_SPEED
        twist.angular.z = self._axis(msg, self.drive_angular_axis) * ANGULAR_SPEED
        self.cmd_vel_pub.publish(twist)

    # -- mode: ARM_JOG ----------------------------------------------------
    def _do_arm_jog(self, msg):
        dt = 1.0 / CONTROL_RATE_HZ
        x = self._axis(msg, self.jog_x_axis)
        y = self._axis(msg, self.jog_y_axis)
        z = self._axis(msg, self.jog_z_axis)

        if x == 0.0 and y == 0.0 and z == 0.0:
            self._time_since_jog_log = 0.0
            return  # sticks centered -- nothing to jog, nothing to publish

        delta = [v * CARTESIAN_SPEED * dt for v in (x, y, z)]
        positions_before_deg = self.arm.positions_deg()
        _, pos_before, pos_after = self.arm.jog_cartesian(delta)
        self._publish_joint_targets()

        # Stick input is continuous (unlike the keyboard's one-shot
        # keypresses), so log at a throttled rate instead of every tick --
        # otherwise this would print ~20 lines/second while jogging.
        self._time_since_jog_log += dt
        if self._time_since_jog_log >= 0.5:
            self._time_since_jog_log = 0.0
            lines = [
                f"[ARM_JOG] stick (x={x:+.2f}, y={y:+.2f}, z={z:+.2f}) -> "
                f"end-effector (m): "
                f"({pos_before.x():.3f}, {pos_before.y():.3f}, {pos_before.z():.3f}) -> "
                f"({pos_after.x():.3f}, {pos_after.y():.3f}, {pos_after.z():.3f})",
            ]
            for name in JOINT_NAMES:
                before = positions_before_deg[name]
                after = math.degrees(self.arm.joint_positions[name])
                lines.append(
                    f"    {name:<24s}: {before:7.2f}° -> {after:7.2f}°  "
                    f"(Δ {after - before:+6.2f}°)"
                )
            self.get_logger().info('\n'.join(lines))

    def _publish_joint_targets(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [self.arm.joint_positions[n] for n in JOINT_NAMES]

        self.arm_command_pub.publish(msg)
        if PUBLISH_TO_JOINT_STATES_FOR_TESTING:
            self.joint_state_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoyCommandMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
