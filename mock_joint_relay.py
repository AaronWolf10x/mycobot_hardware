def generate_launch_description():
    # 1. Define your nodes or components here
    mock_relay_node = Node(
        package='mycobot_280pi', # Replace with your actual package name if different
        executable='mock_joint_relay_node', # Replace with your actual executable name
        name='mock_joint_relay',
        output='screen'
    )

    # 2. Return them inside a LaunchDescription object
    return LaunchDescription([
        mock_relay_node
    ])#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math

class ArmJointStateRelay(Node):
    def __init__(self):
        super().__init__('arm_joint_state_relay')

        # Publisher that robot_state_publisher and RViz listen to
        self.publisher_ = self.create_publisher(JointState, '/joint_states', 10)

        # Timer to publish at 30 Hz
        self.timer = self.create_timer(1.0 / 30.0, self.publish_joint_states)
        self.counter = 0.0

        # Replace these with the exact joint names defined in your arm URDF
        self.joint_names = [
            'g_base_to_joint1',
            'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
            'joint6output_to_joint6'
        ]

        self.get_logger().info('Mock Joint State Publisher running...')

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ''  # Must be empty for JointState messages
        msg.name = self.joint_names

        # Generates a simple smooth sine wave motion to test the joints
        sine_val = 0.5 * math.sin(self.counter)
        msg.position = [
            sine_val,          # joint 1
            sine_val * 0.5,    # joint 2
            -sine_val,         # joint 3
            sine_val * 0.8,    # joint 4
            -sine_val * 0.5,   # joint 5
            sine_val * 0.2,    # joint 6
            sine_val * 0.3
        ]

        self.publisher_.publish(msg)
        self.counter += 0.05

def main(args=None):
    rclpy.init(args=args)
    node = ArmJointStateRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()