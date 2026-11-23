"""Live/recorded fixed-view discovery; no arm command publisher or motion service."""
from concurrent.futures import ThreadPoolExecutor
from collections import deque
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from phyai_discovery_interfaces.srv import GetBlockInfo, ReportView
from tf2_ros import Buffer, TransformListener, TransformException

from phyai_discovery.observations import Capture, Intrinsics, decode_depth, load_capture, load_manifest
from phyai_discovery import Discovery, SearchState, Stationarity
from phyai_discovery.criteria import inspection_targets
from .calibration import load_calibration


def transform_matrix(msg):
    t, q = msg.transform.translation, msg.transform.rotation
    quat = np.array([q.w,q.x,q.y,q.z],dtype=float)
    if not np.isfinite(quat).all() or np.linalg.norm(quat) < 1e-8:
        raise ValueError('Invalid TF quaternion.')
    w,x,y,z = quat/np.linalg.norm(quat)
    matrix = np.eye(4)
    matrix[:3,:3] = [[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                    [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                    [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]]
    matrix[:3,3] = [t.x,t.y,t.z]
    return matrix


def rgb_array(msg):
    if msg.encoding not in ('rgb8','bgr8') or msg.step<msg.width*3 or len(msg.data)!=msg.height*msg.step:
        raise ValueError('RGB capture requires matching rgb8/bgr8 image rows.')
    pixels=np.ndarray((msg.height,msg.width,3),dtype=np.uint8,buffer=msg.data,
                      strides=(msg.step,3,1))
    if msg.encoding=='bgr8':pixels=pixels[:,:,::-1]
    pixels.setflags(write=False)
    return pixels


def camera_intrinsics(info, rectified):
    if not np.isfinite(list(info.d)).all():
        raise ValueError('Invalid camera distortion calibration.')
    if any(abs(d) > 1e-9 for d in info.d) and (not rectified or info.p[0] <= 0 or info.p[5] <= 0):
        raise ValueError('Distorted image requires a rectified stream and calibrated projection matrix.')
    p = info.p if rectified else [0.]*12
    return Intrinsics(info.width,info.height,p[0] if p[0] > 0 else info.k[0],
        p[5] if p[5] > 0 else info.k[4],p[2] if p[0] > 0 else info.k[2],
        p[6] if p[5] > 0 else info.k[5])


class DiscoveryNode(Node):
    def __init__(self, *, node_name='discovery', config_package='phyai_discovery_ros',
                 interface_prefix='discovery', inventory_name='workspace', **kwargs):
        super().__init__(node_name, **kwargs)
        self.declare_parameter('config_path','')
        self.declare_parameter('recording_path','')
        config_path = Path(self.get_parameter('config_path').value or
                           get_package_share_directory(config_package)+'/config/discovery.json')
        self.config = json.loads(config_path.read_text())
        self.input_max_future_skew=float(self.config.get('input_max_future_skew',0.))
        if not np.isfinite(self.input_max_future_skew) or self.input_max_future_skew<0:
            raise ValueError('input_max_future_skew must be finite and nonnegative.')
        from .view_planner import resolve_planner_paths
        resolve_planner_paths(self.config,config_path.parent)
        self.capture_archive=None
        if self.config.get('report_directory'):
            from phyai_discovery.diagnostics import CaptureArchive
            self.capture_archive=CaptureArchive(config_path.parent/self.config['report_directory'])
        recording = self.get_parameter('recording_path').value
        self.recording_entries = None
        if recording:
            manifest,self.recording_base = load_manifest(recording)
            self.recording_entries = manifest['captures']
            self.config.update(map_frame=manifest['map_frame'],viewpoints=manifest['viewpoints'],scope=manifest.get('scope'))
            # Archived transforms already target the manifest's map frame.
            self.config.pop('coordinate_system',None)
            if 'coordinate_system' in manifest:
                self.config['coordinate_system']=manifest['coordinate_system']
            # A historical scene may use a different origin or competition
            # policy. Never apply today's hardware exclusion to its geometry.
            self.config['excluded_regions']=manifest.get('excluded_regions',[])
            if 'stacking_zone_position' in manifest:
                self.config['stacking_zone_position'] = manifest['stacking_zone_position']
        file_calibrations = {}
        if not recording:
            for camera, cfg in self.config['cameras'].items():
                if cfg.get('intrinsics_file'):
                    info, revision = load_calibration(config_path.parent/cfg['intrinsics_file'])
                    file_calibrations[camera] = info
                    cfg['calibration_revision'] = revision
        self.sim_restart_id = None
        self.recording_index = 0
        feature_config = dict(self.config)
        if self.recording_entries is not None:
            feature_config.update(search_criteria=None, inspection_rounds=())
        self.discovery = Discovery(feature_config)
        self.workspace = self.discovery.workspace
        self.search = self.discovery.search
        self.detector = self.discovery.detector
        self.robot_mask_model=None
        self.robot_joint_history=deque(maxlen=600)
        self.robot_base_in_map=None
        self.truth_samples=deque(maxlen=300)
        if self.capture_archive and not recording and self.config.get('ground_truth_topic'):
            self.create_subscription(String,self.config['ground_truth_topic'],self.on_ground_truth,
                QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        robot_mask=self.config.get('view_planner',{}).get('robot_mask',False)
        if robot_mask and not recording:
            from .view_planner import RobotMaskModel
            if isinstance(robot_mask,dict):
                installation_path=config_path.parent/robot_mask['installation_config']
                installation=json.loads(installation_path.read_text())
                resolve_planner_paths(installation,installation_path.parent)
                self.robot_mask_model=RobotMaskModel(installation, use_scene_base=True)
                self.create_subscription(String,robot_mask['scene_topic'],self.on_robot_mask_scene,
                    QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            else:
                self.robot_mask_model=RobotMaskModel(self.config)
        self.rgb_only = self.config.get('detector', {}).get('backend') == 'rgb'
        self.stationarity = Stationarity(self.config.get('stationary_seconds',.5),
            self.config.get('stationary_tolerance',.003),self.config.get('feedback_timeout',1.))
        self.info = file_calibrations
        self.rgb = {}
        self.stream_rgb = {}
        self.overlay_pubs = {camera:self.create_publisher(Image,f'{interface_prefix}/{camera}/overlay',1)
                             for camera in ('overhead','wrist')}
        self.accepted_capture_pub=self.create_publisher(String,f'{interface_prefix}/accepted_capture',
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.masks = {}
        self.dynamic_masks = {}
        self.pending_depth = {}
        self.worker = ThreadPoolExecutor(max_workers=1,thread_name_prefix='discovery_geometry')
        self.future = None
        self.job = None
        self.request_stamp = self.get_clock().now().nanoseconds/1e9
        self.last_error = None
        self.logged_status = None
        self.last_source_received = {}
        self.input_samples = {}
        self.input_totals = {}
        self.detection_samples = {}
        self.detection_totals = {}
        self.last_detection_seconds = None
        self.job_started = None
        self.data_pub = self.create_publisher(String,f'{interface_prefix}/{inventory_name}',1)
        self.progress_pub = self.create_publisher(String,f'{interface_prefix}/search_progress',1)
        self.camera_request_pub = self.create_publisher(String,self.config['sim_camera_request_topic'],1) if self.config.get('sim_camera_request_topic') and self.recording_entries is None else None
        self.state_pub=self.create_publisher(String,f'{interface_prefix}/state',
            QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_service(GetBlockInfo,f'{interface_prefix}/get_block_info',self.get_block_info)
        self.create_service(Trigger,f'{interface_prefix}/capture',self.capture_service)
        self.create_service(ReportView,f'{interface_prefix}/report_view',self.report_view_service)
        self.create_service(Trigger,f'{interface_prefix}/restart_scan',self.restart_service)
        self.create_service(Trigger,f'{interface_prefix}/finish_scan',self.finish_service)
        self.create_service(Trigger,f'{interface_prefix}/end_inspection',self.end_inspection_service)
        self.create_service(Trigger,f'{interface_prefix}/add_view',self.add_view_service)
        if self.recording_entries is None:
            if self.config.get('sim_camera_request_topic'):
                self.create_subscription(String,'sim/scene',self.on_sim_scene,
                    QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.tf = Buffer()
            self.tf_listener = TransformListener(self.tf,self)
            self.create_subscription(JointState,'joint_states',self.on_joints,sensor_qos)
            for camera,cfg in self.config['cameras'].items():
                reliability = self.config.get('image_reliability', 'best_effort')
                if reliability not in ('best_effort', 'reliable'):
                    raise ValueError('Image reliability must be best_effort or reliable.')
                image_qos = QoSProfile(depth=1, reliability=(ReliabilityPolicy.RELIABLE
                    if reliability == 'reliable' else ReliabilityPolicy.BEST_EFFORT))
                if camera not in ('overhead','wrist'):
                    raise ValueError('Only overhead and wrist camera roles are supported.')
                if cfg.get('mask'):
                    self.masks[camera] = np.load(config_path.parent/cfg['mask'],allow_pickle=False)
                if cfg.get('mask_topic'):
                    self.create_subscription(Image,cfg['mask_topic'],
                        lambda msg,camera=camera:self.on_mask(camera,msg),image_qos)
                if cfg.get('stream_rgb_topic'):
                    self.create_subscription(Image,cfg['stream_rgb_topic'],
                        lambda msg,camera=camera:self.on_stream_rgb(camera,msg),image_qos)
                if cfg.get('rgb_topic'):
                    self.create_subscription(Image,cfg['rgb_topic'],
                        lambda msg,camera=camera:self.on_rgb(camera,msg),image_qos)
                if not cfg.get('intrinsics_file') and cfg.get('info_topic'):
                    self.create_subscription(CameraInfo,cfg['info_topic'],
                        lambda msg,camera=camera:self.on_info(camera,msg),image_qos)
                if not self.rgb_only:
                    self.create_subscription(Image,cfg['depth_topic'],
                        lambda msg,camera=camera:self.on_depth(camera,msg),image_qos)
                elif not cfg.get('rgb_topic'):
                    raise ValueError('RGB discovery requires an RGB topic for each camera.')
        self.create_timer(.05,self.tick)
        self.create_timer(.2,self.publish_progress)
        self.create_timer(1.,self.publish)
        self.get_logger().info(f'Discovery awaits {interface_prefix}/capture requests from the app after verified settling.')

    @property
    def discovery_state(self):
        return self.search.state.value

    def state_info(self):
        result=self.search.result
        return {'discovery_state':self.discovery_state,
                'finished':self.search.state==SearchState.FINISHED,
                'session_id':self.workspace.session,'revision':self.workspace.revision,
                'frame_id':self.workspace.frame,
                'search_status':result['status'] if result else ''}

    def get_block_info(self,request,response):
        for name,value in self.state_info().items():
            setattr(response,name,value)
        if request.session_id and request.session_id!=response.session_id:
            response.message='Discovery session changed; query the current session.'
            return response
        if not response.finished:
            response.message='Discovery is still running; block inventory is not final.'
            return response
        snapshot=self.discovery.snapshot()
        if request.block_id:
            tracks=[block for block in snapshot['workspace']['tracks'] if block['id']==request.block_id]
            if not tracks:
                response.message='Unknown confirmed block ID: '+request.block_id
                return response
            snapshot['workspace']['tracks']=tracks
            snapshot['workspace']['hypotheses']=[]
        response.snapshot_json=json.dumps(snapshot,allow_nan=False)
        response.success=True
        response.message='Block information available; inspect search_status and pose uncertainty.'
        return response

    def on_sim_scene(self,msg):
        try:
            scene=json.loads(msg.data)
            restart=scene['restart_id']
            if not isinstance(restart,int) or restart<0:return
            if self.sim_restart_id is not None and restart<self.sim_restart_id:return
            changed=self.sim_restart_id is not None and restart!=self.sim_restart_id
            self.sim_restart_id=restart
            if changed:
                self.restart_service(Trigger.Request(),Trigger.Response())
                self.publish_progress()
                self.publish()
                self.get_logger().info('Simulator restarted; cleared map and restarted discovery.')
            zone=scene.get('stacking',{}).get('zone')
            if zone is not None and self.config.get('import_stacking_zone', True):
                self.discovery.set_stacking_zone(zone['stacking_zone_position'], frame=scene['frame_id'])
        except (ValueError,TypeError,KeyError):pass

    def on_ground_truth(self,msg):
        try:
            scene=json.loads(msg.data)
            sample={key:scene[key] for key in ('simulation_time','frame_id','blocks')}
            if self.truth_samples and sample['simulation_time']<self.truth_samples[-1]['simulation_time']:
                self.truth_samples.clear()
            self.truth_samples.append(sample)
        except (ValueError,TypeError,KeyError):
            pass

    def on_robot_mask_scene(self,msg):
        try:
            scene=json.loads(msg.data)
            position=np.asarray(scene['robot']['base_position'],dtype=float)
            if scene['frame_id']==self.workspace.frame and position.shape==(3,) and np.isfinite(position).all():
                self.robot_base_in_map=position
        except (ValueError,TypeError,KeyError):
            pass

    def on_joints(self,msg):
        positions = dict(zip(msg.name,msg.position))
        try:
            values = [positions[f'joint{i}'] for i in range(1,8)]
        except KeyError:
            self.stationarity.clear()
            return
        self.stationarity.update(values,time.monotonic())
        if self.robot_mask_model is not None and np.isfinite(values).all():
            self.robot_joint_history.append((Time.from_msg(msg.header.stamp).nanoseconds/1e9,values))

    def on_mask(self,camera,msg):
        if msg.encoding != 'mono8' or msg.step < msg.width or len(msg.data) != msg.height*msg.step:
            return
        mask=np.ndarray((msg.height,msg.width),dtype=np.uint8,buffer=bytes(msg.data),strides=(msg.step,1)).astype(bool)
        self.dynamic_masks.setdefault(camera,deque(maxlen=4)).append(
            (Time.from_msg(msg.header.stamp).nanoseconds/1e9,msg.header.frame_id,mask))

    def record_input(self, camera):
        now = time.monotonic()
        self.last_source_received[camera] = now
        self.input_samples.setdefault(camera, deque(maxlen=120)).append(now)
        self.input_totals[camera] = self.input_totals.get(camera, 0)+1

    def on_stream_rgb(self, camera, msg):
        if msg.encoding not in ('rgb8', 'bgr8') or msg.step < msg.width*3 or len(msg.data) != msg.height*msg.step:
            return
        self.record_input(camera)
        rgb = np.ndarray((msg.height,msg.width,3), dtype=np.uint8, buffer=msg.data,
                         strides=(msg.step,3,1))
        if msg.encoding == 'bgr8':
            rgb = rgb[:,:,::-1]
        rgb.setflags(write=False)
        self.stream_rgb[camera] = rgb

    def on_rgb(self,camera,msg):
        if msg.encoding not in ('rgb8','bgr8') or msg.step < msg.width*3 or len(msg.data) != msg.height*msg.step:
            return
        if not self.config['cameras'][camera].get('stream_rgb_topic'):
            self.record_input(camera)
        # Keep just the newest immutable view of the ROS-owned buffer. A capture
        # retains its array's backing buffer while its worker uses it.
        rgb = rgb_array(msg)
        self.rgb[camera] = (Time.from_msg(msg.header.stamp).nanoseconds/1e9,msg.header.frame_id,rgb)
        if self.rgb_only:
            self.on_image(camera, msg)

    def on_info(self,camera,msg):
        if not self.config['cameras'][camera].get('intrinsics_file'):
            self.info[camera] = msg

    def on_depth(self,camera,msg):
        if not self.rgb_only:
            self.on_image(camera, msg)

    def on_image(self,camera,msg):
        now = time.monotonic()
        self.last_source_received[camera] = now
        moving_overhead = (camera == 'overhead' and self.config.get('overlap_overhead_capture')
                           and self.rgb_only and self.robot_mask_model is not None)
        if self.future is not None or (not moving_overhead and not self.stationarity.ready(now)):
            return  # no growing queue and no captures while the arm moves
        if (camera == 'wrist' and self.config.get('overlap_survey_motion')
                and self.search.state == SearchState.STATIONARY):
            # Feedback is already settled. Process this frame instead of
            # discarding it while waiting for the simulation-clock timer.
            self.discovery.tick(now,stationary=True)
        if camera == 'overhead' and self.search.state != SearchState.OVERHEAD:
            return
        if camera == 'wrist' and self.search.state != SearchState.CAPTURE:
            return
        stamp = Time.from_msg(msg.header.stamp).nanoseconds/1e9
        age = self.get_clock().now().nanoseconds/1e9-stamp
        if stamp < self.request_stamp or not -self.input_max_future_skew <= age <= self.config.get('input_max_age',1.):
            self.last_error = ('Image predates capture request or has an invalid/stale timestamp. '
                               f'age={age:.3f}s; allowed future skew={self.input_max_future_skew:.3f}s.')
            if self.pending_depth.get(camera) is msg:self.pending_depth.pop(camera,None)
            return
        try:
            info = self.info.get(camera)
            if info is None:
                raise ValueError('Waiting for matching camera calibration.')
            if info.header.frame_id != msg.header.frame_id or (info.width,info.height) != (msg.width,msg.height):
                raise ValueError('Image and CameraInfo frame/dimensions differ.')
            cfg = self.config['cameras'][camera]
            k = camera_intrinsics(info,cfg.get('rectified',True))
            depth = None if self.rgb_only else decode_depth(
                msg.data,msg.width,msg.height,msg.step,msg.encoding,
                bool(msg.is_bigendian),cfg.get('depth_scale',.001))
            from phyai_discovery.coordinates import source_frame, camera_to_map
            tf = self.tf.lookup_transform(source_frame(self.config),msg.header.frame_id,Time.from_msg(msg.header.stamp))
            digest = hashlib.sha256(json.dumps([list(info.k),list(info.p),list(info.d),
                info.header.frame_id,info.width,info.height,cfg.get('depth_scale',.001),
                cfg.get('calibration_revision','unversioned')]).encode()).hexdigest()[:16]
            rgb = self.rgb.get(camera)
            # A pending RGB-only capture must retain its own pixels while TF
            # catches up; the newest preview may already be a different frame.
            rgb_pixels = (rgb_array(msg) if self.rgb_only else
                rgb[2] if rgb is not None and abs(rgb[0]-stamp) <= .05 and rgb[1] == msg.header.frame_id else None)
            if self.detector.segmenter is not None and rgb_pixels is None:
                raise ValueError('Waiting for capture-matched aligned RGB.')
            mask=self.masks.get(camera)
            if cfg.get('mask_topic'):
                matches=[item for item in self.dynamic_masks.get(camera,())
                         if abs(item[0]-stamp)<=.05 and item[1]==msg.header.frame_id]
                dynamic=min(matches,key=lambda item:abs(item[0]-stamp)) if matches else None
                if dynamic is None:
                    raise ValueError('Waiting for the capture-matched robot mask.')
                mask=dynamic[2] if mask is None else mask | dynamic[2]
            capture = Capture(camera,stamp,msg.header.frame_id,self.workspace.frame,digest,k,depth,
                camera_to_map(self.config,transform_matrix(tf)),self.search.target,roi=tuple(cfg['roi']) if cfg.get('roi') else None,
                mask=mask,rgb=rgb_pixels)
            if self.robot_mask_model is not None:
                projected=self.robot_mask_model.project(capture,self.robot_joint_history,
                                                        base_in_map=self.robot_base_in_map)
                capture=replace(capture,mask=projected if mask is None else mask | projected)
            self.pending_depth.pop(camera,None)
            self.submit(capture)
        except (ValueError,TypeError,KeyError,TransformException) as error:
            self.last_error = str(error)
            if isinstance(error,TransformException) or 'capture-matched' in str(error):
                # Keep the oldest fresh pending frame so TF can catch up rather
                # than chasing the latest future-stamped image indefinitely.
                self.pending_depth.setdefault(camera,msg)
            # Missing TF/calibration can recover before the pending capture times out.

    def submit(self,capture):
        self.job = self.discovery.begin(capture, stationary=True)
        self.job_started = time.monotonic()
        self.future = self.worker.submit(self.detector.detect,self.job.capture)
        self.last_error = None
        if self.config.get('overlap_survey_motion') and capture.camera == 'wrist':
            self.publish_progress()

    def skip_overhead_covered_survey(self):
        info = self.info.get('wrist')
        if info is None or self.robot_mask_model is None:
            return
        try:
            intrinsics = camera_intrinsics(info,self.config['cameras']['wrist'].get('rectified',True))
            intrinsics.validate()
            skipped = self.robot_mask_model.covered_survey_views(
                self.workspace,intrinsics,self.config.get('survey_stages',{}).get('broad',[]),
                self.config['scan']['targets'],self.robot_base_in_map,self.config['detector']['known_block_size'])
            if skipped:
                self.search.skip_covered_survey_views(skipped)
                self.get_logger().info('Overhead already covered survey stops '+', '.join(skipped))
        except (KeyError,ValueError) as error:
            self.get_logger().warning(f'Keeping full wrist survey because coverage could not be checked. {error}')

    def report_view_service(self, request, response):
        try:
            response.success = self.discovery.report_view(request.request_id, request.outcome,
                session_id=request.session_id, now=time.monotonic(), reason=request.reason)
            if response.success and request.outcome == 'arrived':
                self.request_stamp = self.get_clock().now().nanoseconds/1e9
            response.message = (self.search.reason if response.success
                                else 'View report is stale or no longer applicable.')
        except ValueError as error:
            response.success, response.message = False, str(error)
        self.publish_progress()
        return response

    def capture_service(self,request,response):
        try:
            self.discovery.request_capture(time.monotonic())
            self.request_stamp = self.get_clock().now().nanoseconds/1e9
            response.success = True
            response.message = self.search.reason
        except ValueError as error:
            response.success,response.message = False,str(error)
        return response

    def restart_service(self,request,response):
        self.pending_depth.clear()
        self.discovery.reset()
        self.stationarity.clear()
        self.request_stamp = self.get_clock().now().nanoseconds/1e9
        self.recording_index = 0
        self.last_error = None
        response.success,response.message = True,'Scan restarted; old worker results will be discarded.'
        return response

    def finish_service(self,request,response):
        self.discovery.finish()
        response.success,response.message = True,self.search.reason
        return response

    def end_inspection_service(self,request,response):
        if not self.search.inspection_round or self.search.state != SearchState.WRIST:
            response.success,response.message=False,'Targeted inspection can end only between reserve captures.'
            return response
        self.discovery.finish('No clear novel targeted views remain within the per-object inspection budget.',
                              stop_reason='targeted_views_exhausted')
        response.success,response.message=True,self.search.reason
        self.publish_progress()
        self.publish()
        return response

    def add_view_service(self,request,response):
        try:
            self.discovery.add_view()
            response.success,response.message = True,self.search.reason
        except ValueError as error:
            response.success,response.message = False,str(error)
        return response

    def tick(self):
        now = time.monotonic()
        if self.future is None:
            for camera,msg in list(self.pending_depth.items()):
                self.pending_depth.pop(camera,None)
                self.on_image(camera,msg)
        if self.future is not None and self.future.done():
            future,self.future = self.future,None
            job,self.job = self.job,None
            try:
                detection = future.result()
                finished = time.monotonic()
                self.last_detection_seconds = finished-self.job_started
                self.detection_samples.setdefault(job.capture.camera, deque(maxlen=120)).append(finished)
                self.detection_totals[job.capture.camera] = self.detection_totals.get(job.capture.camera, 0)+1
                if self.discovery.complete(job,detection):
                    if job.capture.camera == 'overhead' and self.config.get('skip_overhead_covered_survey'):
                        self.skip_overhead_covered_survey()
                    if self.config.get('overlap_survey_motion'):
                        self.publish_progress()
                    self.publish_overlay(job.capture,detection)
                    self.publish_capture_diagnostics(job.capture,detection)
            except Exception as error:
                if self.discovery.fail(job,error):
                    self.last_error = str(error)
        if self.recording_entries is not None:
            if self.future is None and self.search.state != SearchState.FINISHED:
                if self.recording_index >= len(self.recording_entries):
                    self.discovery.finish('Recorded observations exhausted.')
                    return
                entry = self.recording_entries[self.recording_index]
                self.recording_index += 1
                try:
                    capture = load_capture(entry,self.recording_base)
                    if entry.get('stationary_verified') is not True:
                        raise ValueError('Recording must verify stationary capture.')
                    if self.search.state == SearchState.WRIST:
                        self.discovery.request_capture(now)
                        self.discovery.tick(now,stationary=True)
                    self.submit(capture)
                except (ValueError,KeyError,OSError) as error:
                    self.search.reject(str(error))
        self.discovery.tick(now,stationary=self.stationarity.ready(now) if self.recording_entries is None else True)

    def publish_overlay(self,capture,detection):
        if capture.rgb is None:
            return
        pixels = capture.rgb.copy()
        for candidate in detection.candidates:
            (x0,y0),(x1,y1) = candidate.pixels
            color = [30,220,60] if candidate.resolved else [255,110,20]
            pixels[y0:y0+2,x0:x1+1] = color
            pixels[max(y0,y1-1):y1+1,x0:x1+1] = color
            pixels[y0:y1+1,x0:x0+2] = color
            pixels[y0:y1+1,max(x0,x1-1):x1+1] = color
        msg = Image()
        msg.header.frame_id = capture.optical_frame
        msg.header.stamp = Time(nanoseconds=int(capture.stamp*1e9)).to_msg()
        msg.height,msg.width = pixels.shape[:2]
        msg.encoding,msg.step = 'rgb8',msg.width*3
        msg.data = pixels.tobytes()
        self.overlay_pubs[capture.camera].publish(msg)

    def publish_capture_diagnostics(self,capture,detection):
        if capture.rgb is None:return
        from phyai_discovery.diagnostics import capture_packet
        try:
            packet=capture_packet(capture,detection,getattr(self.detector,'observer',None),
                session_id=self.workspace.session,revision=self.workspace.revision,
                phase='candidate_inspection' if capture.inspection_target else 'overview' if capture.camera=='overhead' else 'survey',
                focus=capture.inspection_target)
            if self.capture_archive:
                archived=packet
                if self.truth_samples:
                    truth=min(self.truth_samples,key=lambda item:abs(item['simulation_time']-capture.stamp))
                    archived={**packet,'ground_truth':truth}
                self.capture_archive.capture(archived)
            self.accepted_capture_pub.publish(String(data=json.dumps(packet,allow_nan=False)))
        except (OSError,ValueError,TypeError) as error:
            self.get_logger().error('Cannot preserve accepted-image diagnostics: '+str(error))

    def camera_rates(self):
        now = time.monotonic()
        def rate(samples):
            if len(samples) < 2 or now-samples[-1] > 2.:
                return 0.
            return (len(samples)-1)/max(samples[-1]-samples[0], 1e-6)
        return {camera: {
            'received_hz': rate(self.input_samples.get(camera, ())),
            'received_frames': self.input_totals.get(camera, 0),
            'detection_hz': rate(self.detection_samples.get(camera, ())),
            'detected_frames': self.detection_totals.get(camera, 0),
        } for camera in self.config.get('cameras', {})}

    def candidate_progress(self):
        confirmed = len(self.workspace.tracks)
        total = len(self.workspace.hypotheses)+len(self.workspace.unverified_hypotheses)
        survey = self.search.index < len(self.search.base_viewpoints) and not self.search.inspection_round
        phase = ('finished' if self.search.state == SearchState.FINISHED else
                 'survey' if survey else 'candidate_inspection')
        return {'phase': phase, 'candidate_count': total,
                'confirmed_blocks': confirmed, 'unconfirmed_candidates': total-confirmed,
                'pending_inspections':len(inspection_targets(self.workspace.hypotheses, self.search.assessment)),
                'count_is_final': False}

    def publish_progress(self):
        self.state_pub.publish(String(data=json.dumps(self.state_info())))
        mode='live' if self.recording_entries is None else 'recorded'
        progress = self.search.progress()
        tracks = self.workspace.hypotheses
        next_survey_pose = None
        if (self.config.get('overlap_survey_motion') and mode == 'live'
                and self.future is not None and self.search.state == SearchState.PROCESSING
                and self.job.generation == self.search.generation
                and self.job.capture.camera == 'wrist' and not self.search.inspection_round
                and self.search.index + 1 < len(self.search.base_viewpoints)):
            next_survey_pose = self.search.base_viewpoints[self.search.index + 1]
        if (self.config.get('overlap_overhead_capture') and self.config.get('overlap_survey_motion')
                and mode == 'live' and self.rgb_only and self.robot_mask_model is not None
                and (self.search.state == SearchState.OVERHEAD or
                     (self.search.state == SearchState.PROCESSING and self.search.processing_camera == 'overhead'))):
            next_survey_pose = self.search.base_viewpoints[0]
        if self.search.state == SearchState.FINISHED:
            final = self.discovery.snapshot()
            progress = final['search']
            tracks = final['workspace']['tracks']+final['workspace']['hypotheses']
        self.progress_pub.publish(String(data=json.dumps({
            'workspace':{'session_id':self.workspace.session,
                         'inspection_targets':inspection_targets(tracks,progress['assessment'])},
            'search':progress,'view_request':self.discovery.view_request(),
            'next_survey_pose':next_survey_pose,
            'robot_base_in_map':self.robot_base_in_map.tolist() if self.robot_base_in_map is not None else None,
            'candidate_focus':self.discovery.focus.snapshot() if self.discovery.focus is not None else None,
            'discovery_progress':self.candidate_progress(),'input_mode':mode,'last_error':self.last_error,
            'sim_restart_id':self.sim_restart_id})))
        if self.camera_request_pub is not None:
            camera=None
            if self.config.get('continuous_camera_stream', False):
                if self.search.state != SearchState.FINISHED:
                    camera = 'overhead' if self.search.state == SearchState.OVERHEAD else 'wrist'
            elif self.stationarity.ready(time.monotonic()) and self.future is None:
                if self.search.state == SearchState.OVERHEAD:camera='overhead'
                elif self.search.state == SearchState.CAPTURE:camera='wrist'
            capture = (self.rgb_only and self.stationarity.ready(time.monotonic())
                       and self.future is None and self.search.state in (SearchState.OVERHEAD, SearchState.CAPTURE))
            self.camera_request_pub.publish(String(data=json.dumps({'camera':camera, 'capture':capture})))

    def publish(self):
        stamp = self.get_clock().now().nanoseconds/1e9 if self.recording_entries is None else None
        envelope = {**self.discovery.snapshot(stamp),
                    'input_mode':'live' if self.recording_entries is None else 'recorded',
                    'sim_restart_id':self.sim_restart_id,
                    'last_error':self.last_error,
                    'discovery_progress':self.candidate_progress(),
                    'camera_rates':self.camera_rates(),
                    'last_detection_seconds':self.last_detection_seconds,
                    'stationary_feedback_ready':self.stationarity.ready(time.monotonic()) if self.recording_entries is None else None,
                    'camera_stream_ages':{camera:time.monotonic()-received for camera,received in self.last_source_received.items()}}
        status = (self.search.state.value,self.search.accepted,self.search.rejected,self.last_error)
        if status != self.logged_status:
            self.get_logger().info(f'{self.search.state.value}: {self.search.reason} '
                                   f'({len(self.workspace.tracks)} tracks); input: {self.last_error or "ready/waiting"}')
            self.logged_status = status
        self.data_pub.publish(String(data=json.dumps(envelope,allow_nan=False)))
        if self.capture_archive:
            try:self.capture_archive.report(envelope)
            except (OSError,ValueError,TypeError) as error:
                self.get_logger().error('Cannot persist discovery report: '+str(error))

    def destroy_node(self):
        self.worker.shutdown(wait=True,cancel_futures=True)
        return super().destroy_node()


def main(args=None):
    from .runtime import run
    run(DiscoveryNode,args)
