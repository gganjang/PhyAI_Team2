import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String


class Heartbeat(Node):
    """Publishes a heartbeat and reports the latest arm joint state from the control PC.

    The app never touches hardware: joint states come from the host stack
    (piper_bridge) over ROS 2, the same way on the real robot and in simulation.
    """

    def __init__(self):
        super().__init__('heartbeat')
        self.latest = None
        self.create_subscription(JointState, 'joint_states', self.on_joint_states, 10)
        self.pub = self.create_publisher(String, 'example/heartbeat', 10)
        self.count = 0
        self.create_timer(1.0, self.tick)

    def on_joint_states(self, msg):
        self.latest = msg

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
    node = Heartbeat()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
