"""Record block_3 pose and joint1-8 from /sim/scene + /joint_states. Usage: record_scene.py out.csv seconds"""
import json
import sys
import time

import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node('scene_recorder')
latest = {}
rows = []


def on_js(msg):
    latest.update(zip(msg.name, msg.position))


def on_scene(msg):
    d = json.loads(msg.data)
    b = next(b for b in d['blocks'] if b['name'] == 'block_3')
    rows.append([d['simulation_time'], *b['position'], *b['quaternion'],
                 *[latest.get(f'joint{i}', float('nan')) for i in range(1, 9)]])


node.create_subscription(JointState, 'joint_states', on_js, 10)
node.create_subscription(String, 'sim/scene', on_scene, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
end = time.monotonic() + float(sys.argv[2])
while time.monotonic() < end:
    rclpy.spin_once(node, timeout_sec=0.05)
with open(sys.argv[1], 'w') as f:
    f.write('t,x,y,z,qw,qx,qy,qz,j1,j2,j3,j4,j5,j6,j7,j8\n')
    for r in rows:
        f.write(','.join(f'{v:.5f}' for v in r) + '\n')
print('rows', len(rows))
