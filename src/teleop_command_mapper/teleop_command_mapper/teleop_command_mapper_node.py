#!/usr/bin/env python3
"""
teleop_command_mapper
======================

One keyboard input, two destinations, switched by a mode key:

  DRIVE mode (default)   -> publishes geometry_msgs/Twist on /cmd_vel
  ARM_JOG mode           -> nudges the end-effector in Cartesian space and
                             publishes the resulting joint angles as a
                             sensor_msgs/JointState on /arm_joint_command
                             (the "dedicated control topic" from the task)

Press TAB to switch modes. Press Ctrl-C to quit.

Drive mode keys (same layout as teleop_twist_keyboard):
    w/x : forward / backward
    a/d : rotate left / right
    s   : stop

Arm-jog mode keys (end-effector Cartesian nudges, in the g_base frame):
    w/x : +X / -X
    a/d : +Y / -Y
    q/z : +Z / -Z

Scoop action key (works in either mode; Neil's note, Oct 23rd/25th meeting):
    g   : excavator-style dig -- sweep the boom/stick/bucket joints down,
          pause, sweep back up

The scoop action is a small non-blocking state machine (ScoopSequencer, in
arm_jog_kinematics.py) shared with the gamepad node, so pressing 'g' here
and pressing the gamepad's scoop button do exactly the same motion. Unlike
the Cartesian jogging below, the scoop does NOT go through the Jacobian
pseudo-inverse -- it drives a few named joints directly (see
BOOM_JOINT/STICK_JOINT/BUCKET_JOINT in arm_jog_kinematics.py), because
that's the only way to get a few joints each making one big, clearly
visible sweep. A Cartesian scoop (tried first) routes through the same
pinv() that handles manual jogging, and pinv() always returns the
*smallest-total-motion* solution -- for this 6-joint/3-DOF arm that means
spreading the scoop's motion as a barely-visible wiggle across all 6
joints, not a clear sweep in a couple of them.

HOW THE ARM JOG MATH WORKS
---------------------------
Cartesian -> joint angles is inverse kinematics. Rather than solving IK from
scratch on every keypress, this does *differential* IK: given the current
joint angles and a small desired Cartesian step, it uses the manipulator
Jacobian's pseudo-inverse to find the joint angle change that produces
(approximately) that Cartesian step. This is the standard technique for
jogging/teleop and is much cheaper than full IK.

The kinematic chain below is built by hand from mycobot_280_pi.urdf's joint
origins/axes rather than parsed from the URDF file at runtime, so there's no
dependency on a URDF-to-KDL parser package being available in this
environment. If the URDF geometry ever changes, update CHAIN_SEGMENTS below
to match.
"""

import math
import sys
import threading

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState

import PyKDL as kdl

from .arm_jog_kinematics import ScoopSequencer

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty


# --- Kinematic chain, hand-built from mycobot_280_pi.urdf ------------------
# Each entry: (joint_name, axis, origin_rpy, origin_xyz, lower_limit, upper_limit)
# 'axis' is always RotZ here because every revolute joint in this URDF uses
# <axis xyz="0 0 1"/>. g_base_to_joint1 is a fixed joint, so it isn't part of
# the movable chain (KDL only needs the moving joints plus the fixed offset,
# which is folded into the first segment below).
JOINT_NAMES = [
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6',
]

JOINT_LIMITS = {
    'joint2_to_joint1': (-2.9321, 2.9321),
    'joint3_to_joint2': (-2.4434, 2.4434),
    'joint4_to_joint3': (-2.6179, 2.6179),
    'joint5_to_joint4': (-2.6179, 2.6179),
    'joint6_to_joint5': (-2.7052, 2.9925),
    'joint6output_to_joint6': (-3.14159, 3.14159),
}


def build_kdl_chain():
    chain = kdl.Chain()
    # g_base -> joint1 (fixed; folded in as a zero-motion segment so the
    # chain's base frame is g_base, matching the URDF/RViz Fixed Frame)
    chain.addSegment(kdl.Segment(
        kdl.Joint(kdl.Joint.Fixed),
        kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0, 0, 0))
    ))
    # joint1 -> joint2
    chain.addSegment(kdl.Segment(
        kdl.Joint(kdl.Joint.RotZ),
        kdl.Frame(kdl.Rotation.Identity(), kdl.Vector(0, 0, 0.13956))
    ))
    # joint2 -> joint3
    chain.addSegment(kdl.Segment(
        kdl.Joint(kdl.Joint.RotZ),
        kdl.Frame(kdl.Rotation.RPY(0, 1.5708, -1.5708), kdl.Vector(0, 0, -0.001))
    ))
    # joint3 -> joint4
    chain.addSegment(kdl.Segment(
        kdl.Joint(kdl.Joint.RotZ),
        kdl.Frame(kdl.Rotation.RPY(0, 0, 0), kdl.Vector(-0.1104, 0, 0))
    ))
    # joint4 -> joint5
    chain.addSegment(kdl.Segment(
        kdl.Joint(kdl.Joint.RotZ),
        kdl.Frame(kdl.Rotation.RPY(0, 0, -1.5708), kdl.Vector(-0.096, 0, 0.06462))
    ))
    # joint5 -> joint6
    chain.addSegment(kdl.Segment(
        kdl.Joint(kdl.Joint.RotZ),
        kdl.Frame(kdl.Rotation.RPY(1.5708, -1.5708, 0), kdl.Vector(0, -0.07318, -0.001))
    ))
    # joint6 -> joint6_flange
    chain.addSegment(kdl.Segment(
        kdl.Joint(kdl.Joint.RotZ),
        kdl.Frame(kdl.Rotation.RPY(-1.5708, 0, 0), kdl.Vector(0, 0.0456, 0))
    ))
    return chain


# --- Cross-platform single-keypress read (same trick as teleop_twist_keyboard) ---
def get_key(settings, timeout=0.1):
    if sys.platform == 'win32':
        import time
        start = time.time()
        while time.time() - start < timeout:
            if msvcrt.kbhit():
                return msvcrt.getwch()
        return ''
    else:
        tty.setraw(sys.stdin.fileno())
        import select
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


def save_terminal_settings():
    return None if sys.platform == 'win32' else termios.tcgetattr(sys.stdin)


DRIVE = 'drive'
ARM_JOG = 'arm_jog'

LINEAR_SPEED = 0.5    # m/s, drive mode
ANGULAR_SPEED = 1.0   # rad/s, drive mode
CARTESIAN_STEP = 0.01  # meters per keypress, arm-jog mode
SCOOP_TICK_HZ = 20.0   # how often the scoop action advances itself

# Set to True to also republish the jogged joint angles on /joint_states so
# you can see the arm move in RViz without a real hardware relay node yet.
# Turn this OFF once a teammate's node (e.g. Jill's hardware relay) is the
# thing driving /joint_states, so the two don't fight over the same topic.
PUBLISH_TO_JOINT_STATES_FOR_TESTING = True


class TeleopCommandMapper(Node):
    def __init__(self):
        super().__init__('teleop_command_mapper')

        self.mode = DRIVE
        self.joint_positions = {name: 0.0 for name in JOINT_NAMES}

        self.chain = build_kdl_chain()
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)
        self.num_joints = self.chain.getNrOfJoints()

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.arm_command_pub = self.create_publisher(JointState, '/arm_joint_command', 10)
        if PUBLISH_TO_JOINT_STATES_FOR_TESTING:
            self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        self.settings = save_terminal_settings()

        self.scoop = ScoopSequencer(self._apply_joint_deltas, on_state_change=self._on_scoop_state_change)
        self.scoop_timer = self.create_timer(1.0 / SCOOP_TICK_HZ, self._scoop_tick)

        self.get_logger().info(
            "teleop_command_mapper ready. TAB = switch mode, currently: DRIVE\n"
            "  Drive:   w/x forward/back, a/d turn, s stop\n"
            "  ArmJog:  w/x +/-X, a/d +/-Y, q/z +/-Z\n"
            "  Scoop:   g (either mode) -- excavator-style boom/stick/bucket dig"
        )

        # Publish the arm's resting pose right away, rather than waiting for
        # the first ARM_JOG keypress. Without this, robot_state_publisher has
        # no value at all for joint2..joint6_flange until you jog for the
        # first time, so RViz shows "No transform from [jointN]" for those
        # links and they don't render (they're not "detached" from the
        # rover -- they just have no known position yet). A periodic timer
        # (rather than a single publish here) also means it doesn't matter
        # which order you start this node and the RViz launch in.
        if PUBLISH_TO_JOINT_STATES_FOR_TESTING:
            self._publish_joint_targets()
            self.joint_state_timer = self.create_timer(0.5, self._publish_joint_targets)

        self._running = True
        self.key_thread = threading.Thread(target=self._key_loop, daemon=True)
        self.key_thread.start()

    # -- mode: DRIVE ----------------------------------------------------
    def handle_drive_key(self, key):
        twist = Twist()
        if key == 'w':
            twist.linear.x = LINEAR_SPEED
        elif key == 'x':
            twist.linear.x = -LINEAR_SPEED
        elif key == 'a':
            twist.angular.z = ANGULAR_SPEED
        elif key == 'd':
            twist.angular.z = -ANGULAR_SPEED
        elif key == 's':
            pass  # zero twist = stop
        else:
            return  # unrecognized key in this mode; don't publish
        self.cmd_vel_pub.publish(twist)

    # -- mode: ARM_JOG ----------------------------------------------------
    def handle_arm_jog_key(self, key):
        if self.scoop.is_active():
            return  # scoop action in progress; ignore manual jogs until it's done
        delta = np.zeros(3)
        axis_label = None
        if key == 'w':
            delta[0] = CARTESIAN_STEP
            axis_label = '+X'
        elif key == 'x':
            delta[0] = -CARTESIAN_STEP
            axis_label = '-X'
        elif key == 'a':
            delta[1] = CARTESIAN_STEP
            axis_label = '+Y'
        elif key == 'd':
            delta[1] = -CARTESIAN_STEP
            axis_label = '-Y'
        elif key == 'q':
            delta[2] = CARTESIAN_STEP
            axis_label = '+Z'
        elif key == 'z':
            delta[2] = -CARTESIAN_STEP
            axis_label = '-Z'
        else:
            return  # unrecognized key in this mode

        self._jog_cartesian(delta, key, axis_label)

    def _current_kdl_joint_array(self):
        q = kdl.JntArray(self.num_joints)
        for i, name in enumerate(JOINT_NAMES):
            q[i] = self.joint_positions[name]
        return q

    def _end_effector_position(self, q):
        frame_out = kdl.Frame()
        self.fk_solver.JntToCart(q, frame_out)
        return frame_out.p  # kdl.Vector

    def _apply_jog_delta(self, delta_xyz):
        """Core differential-IK step for manual Cartesian jogging, factored
        out of _jog_cartesian so the per-keypress logging there stays
        separate from the math. Given a small Cartesian delta (dx, dy, dz
        in meters), computes and applies the resulting joint angle change
        via the Jacobian pseudo-inverse, clamped to each joint's limit,
        and returns (pos_before, pos_after) as kdl.Vector. Not used by the
        scoop action -- see _apply_joint_deltas below for that.
        """
        q_before = self._current_kdl_joint_array()
        pos_before = self._end_effector_position(q_before)

        # Jacobian at the current configuration (6 x N: top 3 rows = linear
        # velocity mapping, bottom 3 = angular -- we only use position here,
        # per the task's "arm end-effector translations").
        jac = kdl.Jacobian(self.num_joints)
        self.jac_solver.JntToJac(q_before, jac)
        j_full = np.array([[jac[r, c] for c in range(self.num_joints)] for r in range(6)])
        j_pos = j_full[0:3, :]

        # Differential IK: joint_delta = pinv(J_pos) @ cartesian_delta
        # NOTE: this arm has 6 joints controlling only 3 position DOFs here,
        # so it's kinematically redundant -- there are infinitely many joint
        # combinations that reach the same point. pinv() picks the one with
        # the smallest total joint movement, which is usually a *little bit
        # of several joints at once*, not one joint moving cleanly. That's
        # expected, not a bug.
        j_pinv = np.linalg.pinv(j_pos)
        joint_delta = j_pinv @ np.asarray(delta_xyz, dtype=float)

        for i, name in enumerate(JOINT_NAMES):
            lower, upper = JOINT_LIMITS[name]
            new_val = self.joint_positions[name] + float(joint_delta[i])
            self.joint_positions[name] = max(lower, min(upper, new_val))

        q_after = self._current_kdl_joint_array()
        pos_after = self._end_effector_position(q_after)
        return pos_before, pos_after

    def _jog_cartesian(self, delta_xyz, key, axis_label):
        positions_before_deg = {n: math.degrees(self.joint_positions[n]) for n in JOINT_NAMES}
        pos_before, pos_after = self._apply_jog_delta(delta_xyz)
        self._publish_joint_targets()

        lines = [
            f"[ARM_JOG] key '{key}' -> end-effector {axis_label} "
            f"{CARTESIAN_STEP * 1000:.0f}mm",
            f"  end-effector (m): "
            f"({pos_before.x():.3f}, {pos_before.y():.3f}, {pos_before.z():.3f}) -> "
            f"({pos_after.x():.3f}, {pos_after.y():.3f}, {pos_after.z():.3f})",
        ]
        for name in JOINT_NAMES:
            before = positions_before_deg[name]
            after = math.degrees(self.joint_positions[name])
            lines.append(
                f"    {name:<24s}: {before:7.2f}° -> {after:7.2f}°  "
                f"(Δ {after - before:+6.2f}°)"
            )
        self.get_logger().info('\n'.join(lines))

    def _publish_joint_targets(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [self.joint_positions[n] for n in JOINT_NAMES]

        self.arm_command_pub.publish(msg)
        if PUBLISH_TO_JOINT_STATES_FOR_TESTING:
            self.joint_state_pub.publish(msg)

    # -- scoop action -----------------------------------------------------
    def _apply_joint_deltas(self, deltas):
        """apply_joint_deltas_fn handed to ScoopSequencer. Unlike manual
        jogging (handle_arm_jog_key -> _jog_cartesian -> _apply_jog_delta,
        all Jacobian-pseudo-inverse-based), the scoop action moves a few
        named joints directly -- see BOOM_JOINT/STICK_JOINT/BUCKET_JOINT
        in arm_jog_kinematics.py -- so this just adds each given radian
        delta straight onto the matching joint, clamped to its limit, and
        republishes. No per-tick log here (unlike _jog_cartesian's
        per-keypress log) -- the scoop ticks ~20 times a second for a
        couple of seconds, so logging every tick would flood the console;
        _on_scoop_state_change below logs the phase transitions instead.
        """
        for name, delta in deltas.items():
            lower, upper = JOINT_LIMITS[name]
            new_val = self.joint_positions[name] + delta
            self.joint_positions[name] = max(lower, min(upper, new_val))
        self._publish_joint_targets()

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

    def _scoop_tick(self):
        if self.scoop.is_active():
            self.scoop.tick(1.0 / SCOOP_TICK_HZ)

    # -- shared key loop ----------------------------------------------------
    def _key_loop(self):
        while self._running and rclpy.ok():
            key = get_key(self.settings, timeout=0.1)
            if key == '':
                continue
            if key == '\t':
                self.mode = ARM_JOG if self.mode == DRIVE else DRIVE
                self.get_logger().info(f'Mode switched to: {self.mode.upper()}')
                if self.mode == DRIVE:
                    self.cmd_vel_pub.publish(Twist())  # make sure the rover stops
                continue
            if key == '\x03':  # Ctrl-C
                self._running = False
                rclpy.shutdown()
                break
            if key == 'g':
                # Scoop action works the same in either mode -- it only
                # touches the arm, never /cmd_vel -- so it's handled here
                # before the DRIVE/ARM_JOG dispatch below rather than
                # inside handle_drive_key/handle_arm_jog_key.
                if not self.scoop.start():
                    self.get_logger().info(
                        'Scoop already in progress -- ignoring key press.')
                continue

            if self.mode == DRIVE:
                self.handle_drive_key(key)
            else:
                self.handle_arm_jog_key(key)

    def destroy_node(self):
        self._running = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopCommandMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
