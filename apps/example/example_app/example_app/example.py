import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class Example(Node):
    """Continuously moves the arm through ROS topics and reports joint feedback.

    Smooth bounded targets make motion visible in the MuJoCo browser stream.
    The same command topic is used by the real robot bridge.
    """

    def __init__(self):
        super().__init__('example')
        self.latest = None
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 10)
        self.pub = self.create_publisher(String, 'example/heartbeat', 10)
        self.count = 0
        self.command_pub = self.create_publisher(JointState, 'joint_ctrl_single', 10)
        self.motion_start = None
        self.initial_positions = None
        self.create_timer(0.02, self.move_arm)  # Smooth target updates at 50 Hz.
        self.create_timer(1.0, self.tick)

    def on_joint_states(self, msg):
        self.latest = msg
        if self.motion_start is None:
            positions = dict(zip(msg.name, msg.position))
            names = [f'joint{i}' for i in range(1, 8)]
            if all(name in positions for name in names):
                self.initial_positions = [positions[name] for name in names]
                self.motion_start = self.get_clock().now()

    def move_arm(self):
        if self.motion_start is None:
            return
        elapsed = (self.get_clock().now() - self.motion_start).nanoseconds / 1e9
        phase = 2.0 * math.pi * elapsed / 12.0
        targets = [
            0.8 * math.sin(phase),
            1.57 + 0.15 * math.sin(1.3 * phase),
            -1.3485 + 0.2 * math.sin(0.9 * phase),
            0.35 * math.sin(1.5 * phase),
            0.15 * math.sin(0.7 * phase),
            0.5 * math.sin(1.1 * phase),
            0.015 * (1.0 - math.cos(phase)),
        ]
        # Ease from the observed pose into the trajectory over two seconds.
        progress = min(1.0, max(0.0, elapsed / 2.0))
        blend = progress * progress * (3.0 - 2.0 * progress)
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [f'joint{i}' for i in range(1, 8)]
        msg.position = [initial + blend * (target - initial)
                        for initial, target in zip(self.initial_positions, targets)]
        self.command_pub.publish(msg)

    def tick(self):
        if self.latest is None:
            state = 'no /joint_states yet'
        else:
            state = ', '.join(f'{n}={p:.2f}' for n, p in zip(self.latest.name, self.latest.position))
        msg = String(data=f'beat {self.count} ({state})')
        self.pub.publish(msg)
        self.get_logger().info(msg.data)
        self.count += 1


def main():
    rclpy.init()
    node = Example()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
