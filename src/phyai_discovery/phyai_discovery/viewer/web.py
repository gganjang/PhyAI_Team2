"""Read-only discovery snapshot viewer; optional external simulator video."""
import errno
import json
from pathlib import Path
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import time
from urllib.parse import urlparse


class WorkspaceWeb:
    def __init__(self,port,simulator_port=0, *, cameras=None, host='127.0.0.1'):
        self.simulator_port=simulator_port
        self.cameras={name: {'topic': topic, 'jpeg': None, 'received': None}
                      for name, topic in (cameras or {}).items()}
        self.latest=None
        self.captures={}
        self.capture_session=None
        self.received=None
        self.lock=threading.Lock()
        page=Path(__file__).with_name('workspace.html').read_bytes()
        owner=self
        class Handler(BaseHTTPRequestHandler):
            def log_message(self,*args):pass
            def do_GET(self):
                path=urlparse(self.path).path
                if path in ('/','/workspace'):
                    data=page;content='text/html; charset=utf-8';code=200
                elif path.startswith('/camera/') and path.endswith('.jpg'):
                    name=path[len('/camera/'):-len('.jpg')]
                    with owner.lock:
                        camera=owner.cameras.get(name)
                        if camera is None:
                            data=b'Unknown camera';content='text/plain';code=404
                        elif camera['received'] is None or time.monotonic()-camera['received']>2.:
                            data=b'Waiting for fresh camera images';content='text/plain';code=503
                        else:
                            data=camera['jpeg'];content='image/jpeg';code=200
                elif path=='/api/workspace':
                    with owner.lock:
                        data=json.dumps({'snapshot':owner.latest,'simulator_port':owner.simulator_port,
                            'cameras': {name: {'topic': camera['topic'],
                                'age': None if camera['received'] is None else time.monotonic()-camera['received']}
                                for name, camera in owner.cameras.items()},
                            'publisher_age':None if owner.received is None else time.monotonic()-owner.received},allow_nan=False).encode()
                    content='application/json';code=200
                elif path=='/api/captures':
                    from phyai_discovery.diagnostics import capture_summary
                    with owner.lock:data=json.dumps([capture_summary(p) for p in owner.captures.values()]).encode()
                    content='application/json';code=200
                elif path.startswith('/api/captures/'):
                    identifier=path.removeprefix('/api/captures/')
                    with owner.lock:packet=owner.captures.get(identifier)
                    data=json.dumps(packet).encode() if packet else b'Unknown accepted capture'
                    content='application/json' if packet else 'text/plain';code=200 if packet else 404
                elif path.startswith('/capture/') and path.endswith('.jpg'):
                    from phyai_discovery.diagnostics import packet_image
                    parts=path.split('/')
                    with owner.lock:packet=owner.captures.get(parts[2]) if len(parts)==4 else None
                    try:
                        if packet is None:raise KeyError('capture')
                        data=packet_image(packet,parts[3].removesuffix('.jpg'));content='image/jpeg';code=200
                    except (KeyError,ValueError,IndexError):
                        data=b'Unknown capture image';content='text/plain';code=404
                else:data=b'Not found';content='text/plain';code=404
                self.send_response(code)
                self.send_header('Content-Type',content)
                self.send_header('Content-Length',str(len(data)))
                self.send_header('Cache-Control','no-store')
                self.end_headers()
                try:self.wfile.write(data)
                except (BrokenPipeError,ConnectionResetError):pass
        try:
            self.server=ThreadingHTTPServer((host,port),Handler)
        except OSError as error:
            if error.errno != errno.EADDRINUSE:
                raise
            raise OSError(error.errno,
                f'Workspace viewer cannot listen on {host}:{port}: address already in use. '
                'Stop the existing listener or choose another port '
                '(ROS launch: viewer_port:=24518; ros2 run: -p http_port:=24518). '
                'To run discovery without its viewer, use with_viewer:=false.') from error
        self.server.daemon_threads=True
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True)
        self.thread.start()

    def update_camera(self, name, jpeg):
        with self.lock:
            self.cameras[name].update(jpeg=jpeg, received=time.monotonic())

    def update_capture(self,packet):
        with self.lock:
            if packet['session_id']!=self.capture_session:
                self.captures={};self.capture_session=packet['session_id']
            self.captures[packet['id']]=packet

    def load_report(self,directory):
        from phyai_discovery.diagnostics import CaptureArchive
        archive=CaptureArchive(directory)
        packets=archive.load_latest()
        for packet in packets:self.update_capture(packet)
        latest=archive.directory/'latest.json'
        if latest.exists():
            session=json.loads(latest.read_text())['session_id']
            self.update(json.loads((archive.directory/session/'report.json').read_text()))

    def update(self,snapshot,*,received=None):
        # Direct Python applications have no ROS query client. A finished core
        # snapshot already contains their final public inventory.
        if 'block_info' not in snapshot and snapshot.get('search', {}).get('state') == 'search_finished':
            snapshot = {**snapshot, 'block_info': {
                'status': 'ready', 'message': 'Final discovery snapshot.',
                'search_status': (snapshot['search'].get('result') or {}).get('status', ''),
                'frame_id': snapshot['workspace']['frame'],
                'revision': snapshot['workspace']['revision'],
                'blocks': snapshot['workspace']['tracks']}}
        with self.lock:
            session=snapshot.get('workspace',{}).get('session_id')
            if session and session!=self.capture_session:
                self.captures={};self.capture_session=session
            self.latest=snapshot;self.received=time.monotonic() if received is None else received

    def close(self):
        self.server.shutdown();self.server.server_close();self.thread.join(timeout=2)
