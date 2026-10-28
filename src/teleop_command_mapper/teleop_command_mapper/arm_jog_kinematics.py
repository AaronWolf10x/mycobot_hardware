#!/usr/bin/env python3
"""
arm_jog_kinematics
===================

Shared Cartesian-jog math for the mycobot arm, used by both input nodes in
this package (keyboard: teleop_command_mapper_node.py, gamepad/on-screen
joystick: joy_command_mapper_node.py). Pulling this out of the keyboard
node means both input methods drive the exact same kinematic chain and
differential-IK math instead of two copies that could quietly drift apart.

The kinematic chain is built by hand from mycobot_280_pi.urdf's joint
origins/axes (see cobot_rover.urdf, which copies the same arm subtree)
rather than parsed from the URDF file at runtime, so there's no dependency
on a URDF-to-KDL parser package being available in this environment. If
the URDF geometry ever changes, update CHAIN_SEGMENTS-equivalent code in
build_kdl_chain() below to match.

Also holds ScoopSequencer, the shared "scoop action" state machine (one
button/key -> excavator-style dig: a few named joints sweep directly,
pause, sweep back) requested in the Oct 23rd/25th meeting notes, so both
input nodes get the same scoop motion instead of two separate
implementations.
"""

import math

import numpy as np
import PyKDL as kdl

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

# -- "Scoop action" (Neil's note, Oct 23rd/25th meeting) --------------------
# Excavator-style dig, done directly in JOINT space -- no inverse
# kinematics at all for this one. Every earlier version of this (straight
# down-and-back; down-then-forward; one diagonal Cartesian sweep) went
# through jog_cartesian()'s Jacobian pseudo-inverse, and that's the actual
# reason it kept looking "stuck in place" on screen: this arm has 6 joints
# controlling only 3 position DOFs, so pinv() always returns the
# *smallest-total-motion* solution -- which in practice means a scoop's
# worth of Cartesian travel gets spread as a barely-visible wiggle across
# all 6 joints at once, instead of a few joints making a clear sweep.
#
# A real excavator doesn't have that problem: its boom, stick, and bucket
# are each driven by their own hydraulic cylinder, so digging is a few
# joints each making one big, distinct motion -- not six joints all
# nudging a little to conspire toward the same point in space. Driving
# BOOM_JOINT/STICK_JOINT/BUCKET_JOINT/WRIST_JOINT directly, by a fixed
# number of degrees each, reproduces exactly that: a plainly visible,
# independent sweep per joint, timed together into one stroke.
#
# BOOM_JOINT and STICK_JOINT sweep the bottom-to-middle of the arm forward
# into the dig. WRIST_JOINT is the one that matters most for the actual
# assignment ("the arm end effector moves near the ground and tries to
# touch the ground"): it's the joint right before the end-effector tip --
# the segment you'd circle in a screenshot as "the top half" -- so it gets
# the biggest sweep of the four, curling that last segment over toward the
# ground as far as a sensible margin from its joint limit allows.
BOOM_JOINT = 'joint3_to_joint2'     # raises/lowers the whole arm
STICK_JOINT = 'joint4_to_joint3'    # curls the forearm in to extend the dig
BUCKET_JOINT = 'joint5_to_joint4'   # tilts mid-wrist, adds to the forward curl
WRIST_JOINT = 'joint6_to_joint5'    # bends the TOP segment (right before the
                                     # end-effector tip) over toward the ground

# How far each joint sweeps for one full dig, in degrees. These (and the
# sign of each) are the numbers to retune once you see it live -- this
# URDF's exact axis conventions weren't re-derived here, so if a joint
# swings the wrong way, flip that joint's sign below. WRIST_JOINT's limit
# is roughly -155 to +171 degrees; 110 leaves some margin while still
# being a large, obvious curl -- push it higher (closer to the limit) if
# it still isn't reaching the ground enough once you see the direction is
# right.
BOOM_SWEEP_DEG = -35.0
STICK_SWEEP_DEG = 50.0
BUCKET_SWEEP_DEG = 30.0
WRIST_SWEEP_DEG = -155.0

SCOOP_SWEEP_DURATION_S = 1.2   # time for the dig stroke (and, mirrored, the lift) to complete
SCOOP_DWELL_S = 0.4            # pause at full dig depth, material "captured", seconds

_SCOOP_TARGET_DEG = {
    BOOM_JOINT: BOOM_SWEEP_DEG,
    STICK_JOINT: STICK_SWEEP_DEG,
    BUCKET_JOINT: BUCKET_SWEEP_DEG,
    WRIST_JOINT: WRIST_SWEEP_DEG,
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


class ArmJogger:
    """Holds the arm's current joint angles and does one differential-IK
    jog step at a time. Both input nodes create one instance of this and
    call jog_cartesian() with whatever small Cartesian step their input
    device asked for this cycle (a fixed step per keypress for the
    keyboard node, a dt-scaled step per control-loop tick for the
    joystick node)."""

    def __init__(self):
        self.chain = build_kdl_chain()
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.jac_solver = kdl.ChainJntToJacSolver(self.chain)
        self.num_joints = self.chain.getNrOfJoints()
        self.joint_positions = {name: 0.0 for name in JOINT_NAMES}

    def current_kdl_joint_array(self):
        q = kdl.JntArray(self.num_joints)
        for i, name in enumerate(JOINT_NAMES):
            q[i] = self.joint_positions[name]
        return q

    def end_effector_position(self, q):
        frame_out = kdl.Frame()
        self.fk_solver.JntToCart(q, frame_out)
        return frame_out.p  # kdl.Vector

    def jog_cartesian(self, delta_xyz):
        """Apply one small Cartesian step (a 3-element iterable: dx, dy, dz
        in meters) via the Jacobian pseudo-inverse, clamp each joint to its
        limit, and return (q_before, pos_before, pos_after) for logging.

        NOTE: this arm has 6 joints controlling only 3 position DOFs here,
        so it's kinematically redundant -- there are infinitely many joint
        combinations that reach the same point. pinv() picks the one with
        the smallest total joint movement, which is usually a *little bit
        of several joints at once*, not one joint moving cleanly. That's
        expected, not a bug.
        """
        q_before = self.current_kdl_joint_array()
        pos_before = self.end_effector_position(q_before)

        jac = kdl.Jacobian(self.num_joints)
        self.jac_solver.JntToJac(q_before, jac)
        j_full = np.array([[jac[r, c] for c in range(self.num_joints)] for r in range(6)])
        j_pos = j_full[0:3, :]

        j_pinv = np.linalg.pinv(j_pos)
        joint_delta = j_pinv @ np.asarray(delta_xyz, dtype=float)

        for i, name in enumerate(JOINT_NAMES):
            lower, upper = JOINT_LIMITS[name]
            new_val = self.joint_positions[name] + float(joint_delta[i])
            self.joint_positions[name] = max(lower, min(upper, new_val))

        q_after = self.current_kdl_joint_array()
        pos_after = self.end_effector_position(q_after)
        return q_before, pos_before, pos_after

    def positions_deg(self):
        return {n: math.degrees(self.joint_positions[n]) for n in JOINT_NAMES}

    def apply_joint_deltas(self, deltas):
        """Directly add joint-space deltas (radians, keyed by joint name),
        clamped to each joint's limit -- no Jacobian, no IK. This is what
        the scoop action (ScoopSequencer, below) drives: a few named
        joints moved directly instead of a Cartesian step solved into all
        six. Manual jogging still goes through jog_cartesian() above;
        this is only for motions where "this specific joint sweeps this
        many degrees" is the actual intent."""
        for name, delta in deltas.items():
            lower, upper = JOINT_LIMITS[name]
            new_val = self.joint_positions[name] + delta
            self.joint_positions[name] = max(lower, min(upper, new_val))


class ScoopSequencer:
    """A small non-blocking state machine for the "scoop action" button,
    modeled on how an excavator actually digs: BOOM_JOINT, STICK_JOINT,
    BUCKET_JOINT, and WRIST_JOINT each sweep directly from their current
    angle to current-angle-plus-their-own-target, all four moving together
    so it reads as one coordinated stroke -- the way a real excavator's
    boom, stick, and bucket cylinders each extend on their own but as part
    of one dig motion. WRIST_JOINT (the top segment, right before the
    end-effector tip) gets by far the largest target, since bending that
    one down is what actually gets the tip toward the ground -- BOOM_JOINT
    and STICK_JOINT mainly reposition the rest of the arm forward into the
    dig. Three phases:

        DESCENDING -> sweep all four joints from 0% to 100% of their
                      target (_SCOOP_TARGET_DEG) -- the dig stroke
        DWELLING   -> pause briefly, as if the material is now captured
        ASCENDING  -> sweep the same four joints back from 100% to 0% --
                      since this is the exact negation of the dig
                      stroke's deltas, it always lands back exactly where
                      DESCENDING started (barring a joint limit clamp
                      along the way, which would be an unusual starting
                      pose to begin with)

    Both input nodes already have their own per-tick "move the arm a
    little" loop (a control-rate timer in the gamepad node; a timer added
    to the keyboard node for this), so this is driven the same way: call
    tick(dt) once per tick, and it's a no-op whenever no scoop is in
    progress. That keeps the whole thing non-blocking -- a plain
    time.sleep() for the multi-second motion would freeze the node's
    keypress/stick handling and ROS callbacks the entire time.

    apply_joint_deltas_fn is whatever the caller uses to add joint-space
    deltas directly -- ArmJogger.apply_joint_deltas for the gamepad node,
    or the keyboard node's own equivalent method. Either way it takes a
    single dict argument ({joint_name: delta_radians, ...}) and applies
    it; that's exactly ArmJogger.apply_joint_deltas's signature, so the
    gamepad node can hand this that method directly.

    on_state_change(old_state, new_state), if given, is called on every
    transition so a node can log "digging" / "lifting" without this
    module needing to know anything about ROS logging.
    """

    IDLE = 'idle'
    DESCENDING = 'descending'
    DWELLING = 'dwelling'
    ASCENDING = 'ascending'

    def __init__(self, apply_joint_deltas_fn, on_state_change=None):
        self._apply_joint_deltas = apply_joint_deltas_fn
        self._on_state_change = on_state_change
        self.state = self.IDLE
        self._progress = 0.0  # 0.0 = start pose, 1.0 = full dig pose
        self._dwell_remaining_s = 0.0

    def is_active(self):
        return self.state != self.IDLE

    def start(self):
        """Begin a scoop if one isn't already running.

        :returns: True if it actually started, False if a scoop was
          already in progress -- so a repeat button press/keypress
          mid-motion is simply ignored rather than restarting or
          queuing another one.
        """
        if self.state != self.IDLE:
            return False
        self._progress = 0.0
        self._set_state(self.DESCENDING)
        return True

    def tick(self, dt):
        """Advance the scoop by one control-loop tick of duration dt
        (seconds). Safe to call every tick unconditionally; it's a no-op
        while IDLE."""
        if self.state == self.DESCENDING:
            self._sweep(dt, +1, self._begin_dwell)
        elif self.state == self.DWELLING:
            self._dwell_remaining_s -= dt
            if self._dwell_remaining_s <= 0.0:
                self._set_state(self.ASCENDING)
        elif self.state == self.ASCENDING:
            self._sweep(dt, -1, lambda: self._set_state(self.IDLE))

    def _sweep(self, dt, sign, on_complete):
        """Advance the shared sweep progress by this tick's share of
        SCOOP_SWEEP_DURATION_S, in the given sign (+1 = toward the full
        dig pose, -1 = back toward the start), moving BOOM_JOINT/
        STICK_JOINT/BUCKET_JOINT together in proportion to each one's own
        target angle. Calls on_complete() once this phase reaches its
        end (progress 1.0 going in, or 0.0 coming back)."""
        target_progress = 1.0 if sign > 0 else 0.0
        step = min(dt / SCOOP_SWEEP_DURATION_S, abs(target_progress - self._progress))
        delta_progress = step * sign
        self._apply_joint_deltas({
            name: math.radians(target_deg) * delta_progress
            for name, target_deg in _SCOOP_TARGET_DEG.items()
        })
        self._progress += delta_progress
        if abs(self._progress - target_progress) <= 1e-9:
            on_complete()

    def _begin_dwell(self):
        self._dwell_remaining_s = SCOOP_DWELL_S
        self._set_state(self.DWELLING)

    def _set_state(self, new_state):
        old_state = self.state
        self.state = new_state
        if self._on_state_change is not None:
            self._on_state_change(old_state, new_state)
