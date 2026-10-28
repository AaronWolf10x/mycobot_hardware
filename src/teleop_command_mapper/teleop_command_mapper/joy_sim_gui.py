#!/usr/bin/env python3
"""
joy_sim_gui
============

An on-screen stand-in for a physical gamepad, for testing
joy_command_mapper_node.py without a USB controller -- exactly the
fallback the team's task notes suggest ("simulate gamepad inputs using
on-screen buttons/sliders in a lightweight Python GUI"). It publishes the
same sensor_msgs/msg/Joy message a real gamepad's `joy_node` would, on
the same /joy topic, so joy_command_mapper_node.py can't tell the
difference.

Sliders:
    Left stick X   -> axes[0]  (drive turn / arm-jog Y, depending on mode)
    Left stick Y   -> axes[1]  (drive forward/back / arm-jog X)
    Right stick Y  -> axes[4]  (arm-jog Z; unused in drive mode)
Each stick slider springs back to 0 when you release the mouse button on
it, the way a real analog stick recenters -- so you don't have to
remember to re-center it yourself.

Buttons:
    Mode Toggle  -> pulses buttons[0] for one publish cycle, which
    joy_command_mapper_node.py reads as a single button press (its
    default mode_button parameter is 0) and uses to switch between
    DRIVE and ARM_JOG, the same as the keyboard node's TAB key.

    Scoop Action -> pulses buttons[1], which joy_command_mapper_node.py
    reads as its scoop_button (default index 1) and uses to run the
    "scoop action": lower the end-effector to the ground, touch down,
    come back up. Works the same whichever mode you're in, same as the
    keyboard node's 'g' key.

Run alongside joy_command_mapper (instead of a real gamepad + the `joy`
package's joy_node):
    ros2 run teleop_command_mapper joy_sim_gui
    ros2 run teleop_command_mapper joy_command_mapper
"""

import threading
import tkinter as tk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

PUBLISH_RATE_HZ = 20.0


class SharedJoyState:
    """Written from the Tkinter main thread, read from the ROS timer
    thread. A lock isn't strictly required (these are simple attribute
    reads/writes under the GIL), but it costs nothing and removes any
    doubt."""

    def __init__(self):
        self.lock = threading.Lock()
        self.left_x = 0.0
        self.left_y = 0.0
        self.right_y = 0.0
        self.mode_pulse = False
        self.scoop_pulse = False

    def axes(self):
        with self.lock:
            return [self.left_x, self.left_y, 0.0, 0.0, self.right_y, 0.0]

    def take_mode_pulse(self):
        with self.lock:
            pulsed = self.mode_pulse
            self.mode_pulse = False
        return pulsed

    def take_scoop_pulse(self):
        with self.lock:
            pulsed = self.scoop_pulse
            self.scoop_pulse = False
        return pulsed


class JoySimPublisher(Node):
    def __init__(self, state: SharedJoyState):
        super().__init__('joy_sim_gui')
        self.state = state
        self.pub = self.create_publisher(Joy, '/joy', 10)
        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._publish)
        self.get_logger().info(
            'joy_sim_gui publishing on /joy. Run joy_command_mapper '
            'alongside this to drive the rover/arm from the on-screen sliders.'
        )

    def _publish(self):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = self.state.axes()
        msg.buttons = [
            1 if self.state.take_mode_pulse() else 0,
            1 if self.state.take_scoop_pulse() else 0,
        ]
        self.pub.publish(msg)


class JoySimApp:
    def __init__(self, root, state: SharedJoyState):
        self.state = state
        root.title('Joystick Simulator (/joy)')

        tk.Label(
            root,
            text='Stand-in for a gamepad -- drag a stick slider, release to recenter.',
            wraplength=320, justify='left',
        ).grid(row=0, column=0, columnspan=2, padx=10, pady=(10, 4), sticky='w')

        self.left_x = self._make_stick(root, 1, 'Left Stick X\n(turn / jog Y)',
                                         orient=tk.HORIZONTAL, setter=self._set_left_x)
        self.left_y = self._make_stick(root, 2, 'Left Stick Y\n(forward-back / jog X)',
                                        orient=tk.HORIZONTAL, setter=self._set_left_y)
        self.right_y = self._make_stick(root, 3, 'Right Stick Y\n(jog Z)',
                                         orient=tk.HORIZONTAL, setter=self._set_right_y)

        tk.Button(
            root, text='Mode Toggle', command=self._pulse_mode, height=2,
        ).grid(row=4, column=0, columnspan=2, padx=10, pady=10, sticky='ew')

        tk.Label(
            root,
            text='Mode Toggle sends one button press, same as TAB on the keyboard node.',
            wraplength=320, justify='left', fg='gray25',
        ).grid(row=5, column=0, columnspan=2, padx=10, pady=(0, 2), sticky='w')

        tk.Button(
            root, text='Scoop Action', command=self._pulse_scoop, height=2,
        ).grid(row=6, column=0, columnspan=2, padx=10, pady=10, sticky='ew')

        tk.Label(
            root,
            text="Scoop Action lowers the end-effector to the ground and back up, "
                 "same as 'g' on the keyboard node. Works in either mode.",
            wraplength=320, justify='left', fg='gray25',
        ).grid(row=7, column=0, columnspan=2, padx=10, pady=(0, 10), sticky='w')

    def _make_stick(self, root, row, label, orient, setter):
        tk.Label(root, text=label, justify='left').grid(
            row=row, column=0, padx=10, pady=6, sticky='w')
        scale = tk.Scale(
            root, from_=-1.0, to=1.0, resolution=0.01, orient=orient,
            length=220, command=lambda v: setter(float(v)),
        )
        scale.set(0.0)
        scale.grid(row=row, column=1, padx=10, pady=6)
        scale.bind('<ButtonRelease-1>', lambda e, s=scale, fn=setter: self._recenter(s, fn))
        return scale

    def _recenter(self, scale, setter):
        scale.set(0.0)
        setter(0.0)

    def _set_left_x(self, v):
        with self.state.lock:
            self.state.left_x = v

    def _set_left_y(self, v):
        with self.state.lock:
            self.state.left_y = v

    def _set_right_y(self, v):
        with self.state.lock:
            self.state.right_y = v

    def _pulse_mode(self):
        with self.state.lock:
            self.state.mode_pulse = True

    def _pulse_scoop(self):
        with self.state.lock:
            self.state.scoop_pulse = True


def main(args=None):
    rclpy.init(args=args)
    state = SharedJoyState()
    node = JoySimPublisher(state)

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    root = tk.Tk()
    JoySimApp(root, state)
    try:
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
