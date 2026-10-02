import os

# Render offscreen on the GPU; no display is needed, so local and remote users see the same browser view
os.environ.setdefault('MUJOCO_GL', 'egl')

import io  # noqa: E402
import threading  # noqa: E402
import time  # noqa: E402
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer  # noqa: E402

import mujoco  # noqa: E402
import rclpy  # noqa: E402
from PIL import Image  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import JointState  # noqa: E402

PAGE = b"""<!doctype html>
<html><head><meta charset="utf-8"><title>PIPER MuJoCo</title>
<style>html,body{margin:0;height:100%;background:#111}
body{display:flex;align-items:center;justify-content:center}
img{max-width:100%;max-height:100%}</style></head>
<body><img src="/stream" alt="PIPER MuJoCo view"></body></html>"""


class FrameHub:
    """Holds the latest JPEG frame and wakes up the browser streams waiting for it."""

    def __init__(self):
        self.cond = threading.Condition()
        self.frame_id, self.jpeg, self.clients = 0, b'', 0

    def publish(self, jpeg):
        with self.cond:
            self.frame_id += 1
            self.jpeg = jpeg
            self.cond.notify_all()

    def wait(self, last_id, timeout=1.0):
        with self.cond:
            self.cond.wait_for(lambda: self.frame_id != last_id, timeout)
            return self.frame_id, self.jpeg


def make_handler(hub):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def do_GET(self):
            if self.path == '/':
                self.send_response(200)
                self.send_header('Content-Type', 'text/html')
                self.send_header('Content-Length', str(len(PAGE)))
                self.end_headers()
                self.wfile.write(PAGE)
            elif self.path == '/stream':
                self.stream()
            else:
                self.send_error(404)

        def stream(self):
            """MJPEG stream: the browser replaces the image with each new part."""
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-cache')
            self.end_headers()
            with hub.cond:
                hub.clients += 1
                # Rendering pauses while nobody watches, so skip the stale frame and wait for a fresh one
                last_id = hub.frame_id
            try:
                while True:
                    frame_id, jpeg = hub.wait(last_id)
                    if frame_id == last_id or not jpeg:
                        continue
                    last_id = frame_id
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n'
                                     % len(jpeg) + jpeg + b'\r\n')
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                with hub.cond:
                    hub.clients -= 1

    return Handler


class PiperMujocoSim(Node):
    """MuJoCo stand-in for the control PC host stack (piper_bridge).

    Publishes the arm's joint states and drives the simulated arm from joint position
    commands, over the same topics apps use on the real robot. The scene is streamed
    to a browser at http://<dev-pc>:<http_port>.
    """

    def __init__(self):
        super().__init__('piper_mujoco_sim')
        self.declare_parameter('model_path', '/opt/mujoco_menagerie/agilex_piper/scene.xml')
        self.declare_parameter('state_topic', 'joint_states')
        self.declare_parameter('command_topic', 'joint_ctrl_single')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('http_port', 23517)
        self.declare_parameter('view_fps', 30.0)
        self.declare_parameter('view_width', 960)
        self.declare_parameter('view_height', 720)
        param = lambda name: self.get_parameter(name).value

        self.model = mujoco.MjModel.from_xml_path(param('model_path'))
        self.data = mujoco.MjData(self.model)
        home = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, 'home')
        if home >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, home)
        mujoco.mj_forward(self.model, self.data)

        self.joints = [self.model.joint(i) for i in range(self.model.njnt)]
        # Command names -> actuator index; the gripper actuator also answers to 'joint7'
        self.actuators = {self.model.actuator(i).name: i for i in range(self.model.nu)}
        if 'gripper' in self.actuators:
            self.actuators.setdefault('joint7', self.actuators['gripper'])

        self.lock = threading.Lock()
        self.pub = self.create_publisher(JointState, param('state_topic'), 10)
        self.create_subscription(JointState, param('command_topic'), self.on_command, 10)
        self.publish_period = 1.0 / param('publish_rate')

        self.hub = FrameHub()
        self.view_period = 1.0 / param('view_fps')
        self.view_size = (param('view_width'), param('view_height'))
        self.server = ThreadingHTTPServer(('0.0.0.0', param('http_port')), make_handler(self.hub))
        self.server.daemon_threads = True

        self.get_logger().info(
            f"command: /{param('command_topic')}  state: /{param('state_topic')}  "
            f"view: http://localhost:{param('http_port')} (or http://<dev-pc-ip>:{param('http_port')})")
        for target in (self.run_physics, self.run_view, self.server.serve_forever):
            threading.Thread(target=target, daemon=True).start()

    def on_command(self, msg):
        with self.lock:
            for name, position in zip(msg.name, msg.position):
                if name in self.actuators:
                    self.data.ctrl[self.actuators[name]] = position

    def publish_state(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            for joint in self.joints:
                msg.name.append(joint.name)
                msg.position.append(float(self.data.qpos[joint.qposadr[0]]))
                msg.velocity.append(float(self.data.qvel[joint.dofadr[0]]))
        self.pub.publish(msg)

    def run_physics(self):
        """Steps physics in real time and publishes joint states at publish_rate."""
        dt = self.model.opt.timestep
        steps_per_publish = max(1, round(self.publish_period / dt))
        start, step = time.monotonic(), 0
        while rclpy.ok():
            with self.lock:
                mujoco.mj_step(self.model, self.data)
            step += 1
            if step % steps_per_publish == 0:
                self.publish_state()
            ahead = start + step * dt - time.monotonic()
            if ahead > 0:
                time.sleep(ahead)

    def run_view(self):
        """Renders JPEG frames for the browser view, only while someone is watching."""
        width, height = self.view_size
        self.model.vis.global_.offwidth = max(self.model.vis.global_.offwidth, width)
        self.model.vis.global_.offheight = max(self.model.vis.global_.offheight, height)
        try:
            renderer = mujoco.Renderer(self.model, height, width)
        except Exception as e:  # e.g. no GPU / EGL in the container
            self.get_logger().error(f'browser view disabled, offscreen rendering failed: {e}')
            return
        camera = mujoco.MjvCamera()
        camera.azimuth, camera.elevation, camera.distance = 120.0, -20.0, 1.2
        camera.lookat[:] = (0.0, 0.0, 0.25)
        while rclpy.ok():
            started = time.monotonic()
            if self.hub.clients > 0:
                with self.lock:
                    renderer.update_scene(self.data, camera)
                buf = io.BytesIO()
                Image.fromarray(renderer.render()).save(buf, format='JPEG', quality=80)
                self.hub.publish(buf.getvalue())
            time.sleep(max(0.0, self.view_period - (time.monotonic() - started)))
        renderer.close()

    def destroy_node(self):
        self.server.shutdown()
        super().destroy_node()


def main():
    rclpy.init()
    node = PiperMujocoSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
