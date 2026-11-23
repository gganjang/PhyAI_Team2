"""Shared PIPER joint-position scan execution for simulation and hardware bridges."""
import json
import math
import time
from pathlib import Path

import numpy as np
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from phyai_discovery_interfaces.srv import ReportView


NAMES = [f'joint{i}' for i in range(1,8)]


def smooth_target(initial, target, elapsed, duration, *, jerk_limited=False):
    t = min(1.,max(0.,elapsed/duration))
    # Seventh-order blend has zero velocity, acceleration AND jerk at both
    # ends. Keep the existing brief-scan profile unless explicitly selected.
    blend = t**4*(35-84*t+70*t*t-20*t**3) if jerk_limited else t*t*(3-2*t)
    return [float(a+blend*(b-a)) for a,b in zip(initial,target)]


def move_duration(initial, target, minimum, max_speed, max_acceleration, max_jerk=None):
    """Bound analytic peak derivatives of the selected joint trajectory."""
    distance = max(abs(a-b) for a,b in zip(initial[:6],target[:6]))
    if max_jerk is not None:
        return max(minimum, 2.1875*distance/max_speed,
                   math.sqrt((84*math.sqrt(5)/25)*distance/max_acceleration),
                   (52.5*distance/max_jerk)**(1/3))
    return max(minimum, 1.5*distance/max_speed,
               math.sqrt(6*distance/max_acceleration))


class ScanNode(Node):
    def __init__(self, *, node_name='discovery_scan', config_package='phyai_discovery_ros',
                 config_name='discovery.json', scan_key='scan', interface_prefix='discovery', **kwargs):
        super().__init__(node_name, **kwargs)
        self.declare_parameter('config_path','')
        self.declare_parameter('command_topic','joint_ctrl_single')
        self.declare_parameter('joint_states_topic','joint_states')
        self.command_topic=self.get_parameter('command_topic').value
        self.progress_topic=f'{interface_prefix}/search_progress'
        config_path=Path(self.get_parameter('config_path').value or
            get_package_share_directory(config_package)+'/config/'+config_name)
        config=json.loads(config_path.read_text())
        from .view_planner import resolve_planner_paths
        resolve_planner_paths(config,config_path.parent)
        scan=config.get(scan_key)
        if not isinstance(scan,dict):
            raise ValueError('Automatic scanning requires a '+scan_key+' configuration with installation joint targets.')
        self.config=config
        self.scan=scan
        self.limits=scan.get('joint_limits')
        if scan_key=='scan':
            limits=np.asarray(self.limits,dtype=float)
            if limits.shape!=(6,2) or not np.isfinite(limits).all() or np.any(limits[:,0]>=limits[:,1]):
                raise ValueError('Scan requires six finite joint_limits [lower, upper] pairs.')
        self.cable_yaw_limit=scan.get('cable_yaw_limit')
        if self.cable_yaw_limit is not None:
            if (not isinstance(self.cable_yaw_limit,(int,float)) or isinstance(self.cable_yaw_limit,bool)
                    or not math.isfinite(self.cable_yaw_limit) or not 0<self.cable_yaw_limit<=math.pi
                    or self.limits[0][0]<-self.cable_yaw_limit or self.limits[0][1]>self.cable_yaw_limit):
                raise ValueError('Base joint limits must stay within the cable limit (at most +/-180 deg).')
        self.targets=scan['targets']
        self.survey_labels=set(config['viewpoints'])
        self.prepositioning=False
        if scan_key=='scan' and scan.get('targeted_inspection', False):
            raise ValueError('Use shared candidate_focus settings and a plan_view robot adapter.')
        self.planner=None
        self.request_plans={}
        self.request_routes={}
        self.selected_targets={}
        self.inspection_focus=None
        self.duration=float(scan.get('move_seconds',4.))
        self.approach_duration=float(scan.get('approach_seconds',8.))
        self.settle_seconds=float(scan.get('settle_seconds',.7))
        self.max_speed=float(scan.get('max_joint_speed',1.2))
        self.max_acceleration=float(scan.get('max_joint_acceleration',3.))
        if not all(math.isfinite(v) and v>0 for v in (self.duration,self.approach_duration,
                       self.settle_seconds,self.max_speed,self.max_acceleration)):
            raise ValueError('Scan timing must be positive.')
        self.motion_profiles=scan.get('motion_profiles',{})
        for phase in self.motion_profiles:self.motion_settings(phase)
        labels = config['viewpoints'] + [name for views in config.get('inspection_rounds',[]) for name in views]
        focus_enabled = config.get('candidate_focus', {}).get('enabled', False)
        required = set(config['viewpoints']) if focus_enabled else set(labels)
        if not required.issubset(self.targets) or not set(self.targets).issubset(labels):
            raise ValueError('Survey poses are required; legacy fixed scans also need every reserve pose.')
        for values in self.targets.values():
            if len(values)!=7 or not np.isfinite(values).all():
                raise ValueError('Scan targets must contain seven finite joints.')
        self.actual=None;self.feedback_received=None
        self.discovery=None;self.discovery_received=None
        self.session=None;self.active=None;self.initial=None;self.started=None
        self.request_key=None;self.active_request_id=None
        self.settled_since=None;self.capture_future=None;self.fault=None
        self.moved=False;self.fault_session=None
        self.approach_waypoint=None
        self.remaining_waypoints=[]
        self.inspection_trajectory=None
        self.last_motion_command=None
        self.wait_reason='Waiting for bridge feedback and discovery startup.'
        self.logged_wait=None
        self.client=self.create_client(Trigger,f'{interface_prefix}/capture')
        self.view_client=self.create_client(ReportView,f'{interface_prefix}/report_view')
        self.view_report_future=None
        self.finish_client=self.create_client(Trigger,f'{interface_prefix}/end_inspection')
        self.finish_future=None
        self.pub=self.create_publisher(JointState,self.command_topic,1)
        self.status_pub=self.create_publisher(String,f'{interface_prefix}/scan_status',1)
        self.stop_client=self.create_client(Trigger,f'{interface_prefix}/finish_scan')
        self.create_service(Trigger,f'{interface_prefix}/stop_scan',self.stop_service)
        self.create_subscription(JointState,self.get_parameter('joint_states_topic').value,self.on_joints,
            QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(String,self.progress_topic,self.on_discovery,1)
        self.create_timer(.02,self.tick)
        self.create_timer(.5,self.publish_status)
        self.get_logger().info('PIPER scan enabled using JointState commands and measured arrival feedback.')

    def clear_motion(self):
        self.prepositioning=False
        self.active=self.initial=self.started=self.settled_since=None
        self.capture_future=None
        self.request_key=None
        self.active_request_id=None
        self.approach_waypoint=None
        self.remaining_waypoints=[]

    def motion_time(self):
        return time.monotonic()

    def motion_settings(self,phase):
        settings={'move_seconds':self.duration,'settle_seconds':self.settle_seconds,
                  'max_joint_speed':self.max_speed,'max_joint_acceleration':self.max_acceleration,
                  'max_joint_jerk':self.scan.get('max_joint_jerk')}
        profile=self.motion_profiles.get(phase,{})
        if not isinstance(profile,dict) or set(profile)-set(settings):
            raise ValueError('Unknown motion profile setting for '+str(phase))
        settings.update(profile)
        if not all(isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v) and v>0
                   for k,v in settings.items() if not (k=='max_joint_jerk' and v is None)):
            raise ValueError('Motion profile values must be finite and positive.')
        settings['max_joint_speed']=min(settings['max_joint_speed'],self.max_speed)
        settings['max_joint_acceleration']=min(settings['max_joint_acceleration'],self.max_acceleration)
        return settings

    def environment_ready(self, now):
        return True

    def target_limits(self):
        return self.limits

    def accept_progress(self, data):
        return True

    def can_command(self):
        if not self.get_subscriptions_info_by_topic(self.command_topic):
            return False
        own=(self.get_name(),self.get_namespace())
        publishers=self.get_publishers_info_by_topic(self.command_topic)
        # ROS permits duplicate node names; matching names alone cannot prove
        # exclusive ownership of the physical command stream.
        if len(publishers)!=1 or (publishers[0].node_name,publishers[0].node_namespace)!=own:
            return False
        # Competing discovery sessions can repeatedly reset a trajectory even
        # with just one scan node. Wait until progress has a single producer.
        return len(self.get_publishers_info_by_topic(self.progress_topic))<=1

    def stop_service(self, request, response):
        self.fault_stop('Scan cancelled by client.')
        if self.stop_client.service_is_ready():
            self.stop_future=self.stop_client.call_async(Trigger.Request())
        response.success=True
        response.message='Scan command stream stopped; bridge remains responsible for hardware stop handling.'
        return response

    def on_joints(self,msg):
        values=dict(zip(msg.name,msg.position))
        if all(name in values and math.isfinite(values[name]) for name in NAMES):
            self.actual=[values[name] for name in NAMES]
            self.feedback_received=time.monotonic()
        else:
            self.actual=None
            if self.active:self.fault_stop('Incomplete or invalid PIPER feedback.')

    def on_discovery(self,msg):
        try:
            data=json.loads(msg.data)
            if data.get('input_mode')!='live':return
            if not self.accept_progress(data):return
            session=data['workspace']['session_id']
            if self.session != session:
                self.clear_motion();self.moved=False
                self.selected_targets={};self.inspection_focus=None
                self.request_plans={};self.view_report_future=None
                self.request_routes={}
                self.finish_future=None
                self.inspection_trajectory=None
                self.last_motion_command=None
                if self.planner is not None:self.planner.reset()
                if self.fault_session != session:self.fault=None
            self.session=session
            self.discovery=data;self.discovery_received=time.monotonic()
        except (ValueError,KeyError,TypeError):
            self.discovery=None
            if self.active:self.fault_stop('Invalid discovery progress.')

    def fault_stop(self,reason):
        self.fault=reason;self.fault_session=self.session
        self.clear_motion()
        self.last_motion_command=None
        self.get_logger().error(reason)
        # A measured-position hold uses the agreed position-command contract.
        # Hardware emergency stopping remains the bridge's responsibility.
        if (self.actual is not None and self.feedback_received is not None
                and 0<=time.monotonic()-self.feedback_received<=1. and self.can_command()):
            self.command(self.actual)

    def command(self,target):
        msg=JointState()
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.name=NAMES;msg.position=target
        self.pub.publish(msg)

    def plan_view(self, request, current):
        """Robot adapter hook: return feasible joints for the requested target.

        Robot adapters own IK, collision/visibility checks and view geometry only.
        Candidate selection and budgets are owned by shared Discovery.
        """
        label=request['viewpoint']
        if request.get('focus') is None:
            target=self.targets[label]
            # Survey paths were checked during generation. The host bridge
            # owns execution safeguards for the approach from measured joints.
            if self.planner is not None:self.planner.remember(label,target)
            return target
        from phyai_discovery.geometry import footprint_inside,outside_excluded_regions
        focus=request['focus']
        if not footprint_inside(focus.get('position'),focus.get('size'),
                focus.get('quaternion'),scope=self.config.get('scope'),
                polygon=self.config.get('table_polygon')):
            self.inspection_focus=None
            self.get_logger().warning('Ignoring inspection target outside the tabletop or missing footprint geometry: '+focus.get('candidate_id',focus.get('id','unknown')))
            return None
        if not outside_excluded_regions(focus['position'][:2],self.config.get('excluded_regions')):
            self.inspection_focus=None
            self.get_logger().warning('Ignoring inspection target in an excluded search region.')
            return None
        if self.planner is None:
            from .view_planner import create_view_planner
            self.planner=create_view_planner(self.config,self.targets)
            for name,joints in self.selected_targets.items():self.planner.remember(name,joints)
        from phyai_discovery.coordinates import target_to_source, map_origin
        target=target_to_source(self.config,focus)
        origin=map_origin(self.config)
        if isinstance(self.config.get('view_planner',{}).get('robot_mask'),dict):
            origin=-np.asarray(self.discovery['robot_base_in_map'])
            target={**focus,'position':(np.asarray(focus['position'])+origin).tolist()}
        if request.get('companions'):
            target['companions']=[{**peer,'position':(np.asarray(peer['position'])+origin).tolist()}
                                  for peer in request['companions']]
        plan=self.planner.choose([target],current,label)
        if plan:
            # Planner geometry stays in the arm frame; public targets and logs
            # use the same tabletop map as the measured inventory.
            plan={**plan,'position':(np.asarray(plan['position'])-origin).tolist()}
        self.inspection_focus=plan
        if plan:
            self.request_routes[request.get('request_id')]=plan.get('waypoints',[])
            shared=plan.get('shared_candidates',[])
            detail=f'; also targeting {", ".join(shared)}' if shared else ''
            self.get_logger().info(f'Inspection {label}: focus {plan["focus"]} at {plan["position"]}{detail}')
            return plan['joints']
        return None

    def select_target(self,label):
        request = self.discovery.get('view_request')
        if request is None:
            return self.targets[label]
        if (request['session_id'] != self.session or request['viewpoint'] != label):
            self.fault_stop('Discovery view request does not match the active scan.')
            return None
        key = request['request_id']
        if key in self.request_plans:
            return self.request_plans[key]
        try:
            target = self.plan_view(request, self.actual)
        except (NotImplementedError, ValueError, OSError) as error:
            self.fault_stop(str(error))
            return None
        if target is not None:
            self.request_plans[key] = list(target)
            self.selected_targets[label] = list(target)
        return target

    def report_view(self, request, outcome, reason=''):
        if not self.view_client.service_is_ready():
            self.wait_reason = 'Waiting for discovery view-report service.'
            return None
        return self.view_client.call_async(ReportView.Request(
            session_id=request['session_id'], request_id=request['request_id'],
            outcome=outcome, reason=reason))

    def tick(self):
        now=time.monotonic()
        if self.fault:return
        if not self.environment_ready(now):return
        if self.actual is None:
            if self.active:self.fault_stop('Joint feedback is unavailable.')
            self.wait_reason='Waiting for complete /joint_states.';return
        if self.cable_yaw_limit is not None and abs(self.actual[0])>self.cable_yaw_limit:
            self.fault_stop('Measured base angle exceeds the cable limit; reposition before scanning.');return
        if self.discovery is None:
            if self.active:self.fault_stop('Discovery progress is unavailable.')
            self.wait_reason='Waiting for discovery search progress.';return
        if not self.can_command():
            self.wait_reason='Waiting for a bridge subscriber and exclusive command publishing.'
            if self.active:self.fault_stop('Command publishing conflict or missing bridge; scan halted.')
            return
        if now-self.feedback_received>1. or now-self.discovery_received>3.:
            self.wait_reason='Waiting for fresh feedback and search progress.'
            if self.active:self.fault_stop('Joint feedback or discovery became stale.')
            return
        progress=self.discovery['search']
        if (isinstance(self.config.get('view_planner',{}).get('robot_mask'),dict)
                and self.discovery.get('robot_base_in_map') is None):
            self.wait_reason='Waiting for the simulator robot base position.'
            return
        if progress['state']=='search_finished':
            if self.active:self.command(self.last_motion_command if self.request_key is not None and self.last_motion_command is not None else self.actual)
            self.wait_reason='Search completed.'
            self.clear_motion();return
        if self.finish_future is not None:
            self.wait_reason='Waiting for targeted inspection to finish.'
            if self.finish_future.done():
                try:
                    result=self.finish_future.result()
                    if not result.success:self.fault_stop('Inspection stop rejected: '+result.message)
                except Exception as error:self.fault_stop(str(error))
            return
        target_name=progress['target']
        view_request=self.discovery.get('view_request')
        request_id=view_request['request_id'] if view_request is not None else None
        ahead=self.discovery.get('next_survey_pose') if self.config.get('overlap_survey_motion') else None
        overhead_start=(self.config.get('overlap_overhead_capture') and progress['state']=='capture_overhead')
        if ahead not in self.survey_labels or (progress['state']!='detect_and_merge' and not overhead_start):
            ahead=None
        if self.prepositioning:
            if progress['state']=='wait_for_wrist_view' and target_name==self.active:
                # Adopt the in-flight trajectory under the actual capture request.
                self.prepositioning=False
                self.active_request_id=request_id
            else:
                # Finish the checked leg, including when discovery needs to retry
                # the preceding image. Never report arrival for a predicted view.
                ahead=self.active
        if ahead:
            target_name=ahead
            request_id=None
            view_request=None
        if self.view_report_future is not None:
            self.wait_reason='Waiting for discovery to select the next view.'
            if self.view_report_future.done():
                try:self.view_report_future.result()
                except Exception as error:self.fault_stop(str(error))
                self.view_report_future=None
            return
        if self.active is not None and (target_name!=self.active or request_id!=self.active_request_id):
            # A new request can reuse a slot after an unreachable candidate.
            # Retire the old motion before accepting its replacement.
            if self.request_key is None:
                self.fault_stop('View request changed before measured arrival; scan halted.')
                return
            self.command(self.last_motion_command if self.last_motion_command is not None else self.actual)
            self.clear_motion()
        if self.capture_future is not None:
            self.wait_reason='Waiting for capture acknowledgement.'
            if self.capture_future.done():
                try:
                    result=self.capture_future.result()
                    if not result.success:
                        if view_request is None:self.fault_stop('Capture request rejected: '+result.message)
                        else:self.clear_motion()
                except Exception as error:self.fault_stop(str(error))
                self.capture_future=None
            return
        if not ahead and progress['state']!='wait_for_wrist_view':
            self.wait_reason='Discovery: '+progress['state'];return
        if not ahead and self.config.get('candidate_focus', {}).get('enabled', False) and view_request is None:
            self.wait_reason='Waiting for the shared discovery view request.'
            return
        key=(self.session,progress['viewpoint_index'],progress['rejected'],request_id)
        if getattr(self,'request_key',None)==key:return
        if getattr(self,'request_key',None) is not None:
            self.clear_motion()
        if target_name not in self.targets and not (view_request and view_request.get('focus')):
            self.fault_stop('No configured scan pose for '+target_name);return
        target=self.targets[target_name] if ahead else self.select_target(target_name)
        if self.fault:return
        if target is None and view_request is not None:
            reason=(getattr(self.planner,'last_failure',None)
                    or 'Robot adapter found no feasible collision-free, visible view.')
            self.get_logger().warning(f'No inspection plan for {target_name}: {reason}')
            self.view_report_future=self.report_view(view_request, 'unreachable',reason)
            return
        if target is None:
            self.wait_reason='Targeted views exhausted; preserving unresolved evidence.'
            if self.finish_client.service_is_ready():
                self.finish_future=self.finish_client.call_async(Trigger.Request())
            return
        if self.active is None:
            # Planning can block this single-threaded executor long enough for
            # the input checks at tick entry to expire. Keep the cached plan,
            # but let subscriptions refresh before starting a command stream.
            now=time.monotonic()
            if not self.environment_ready(now):return
            if now-self.feedback_received>1. or now-self.discovery_received>3.:
                self.wait_reason='Waiting for fresh feedback and search progress.'
                return
            limits=self.target_limits()
            if any(not low<=angle<=high for angle,(low,high) in zip(target[:6],limits)) or not 0<=target[6]<=.035:
                self.fault_stop('Scan target exceeds PIPER joint limits.');return
            route=self.request_routes.get(request_id,[])
            if route and (not np.allclose(route[-1],target) or any(
                    len(q)!=7 or not np.isfinite(q).all() or not 0<=q[6]<=.035
                    or any(not low<=v<=high for v,(low,high) in zip(q[:6],limits)) for q in route)):
                self.fault_stop('Inspection reposition route exceeds joint limits or misses its target.');return
            anchor=self.last_motion_command if self.last_motion_command is not None else self.actual
            if (max(abs(a-b) for a,b in zip(anchor[:6],self.actual[:6]))>=.025
                    or abs(anchor[6]-self.actual[6])>=.002):
                self.fault_stop('Arm moved away from the held command between views; scan halted.')
                return
            self.active=target_name;self.active_request_id=request_id;self.initial=list(anchor)
            self.prepositioning=bool(ahead)
            self.started=self.motion_time();self.settled_since=None
            broad=self.config.get('survey_stages',{}).get('broad',[])
            if not self.moved and self.scan.get('raise_before_sweep',False) and target_name in broad:
                # Raise at the measured base angle. Only after this posture
                # settles may the base rotate toward the first survey view.
                self.approach_waypoint=[self.actual[0],*target[1:]]
            phase=view_request.get('phase','survey') if view_request else 'survey'
            if phase=='survey':
                stages=self.config.get('survey_stages',{})
                if target_name in stages.get('near_post',[]):
                    phase='near_post'
                elif target_name in stages.get('coverage',[]):
                    phase='coverage'
            self.active_phase=phase
            if phase=='candidate_inspection' and len(route)>1:
                self.approach_waypoint=list(route[0])
                self.remaining_waypoints=[list(q) for q in route[1:-1]]
            timing=self.motion_settings(phase)
            self.active_settle_seconds=timing['settle_seconds']
            self.active_max_jerk=timing['max_joint_jerk']
            self.active_duration=move_duration(self.initial,self.approach_waypoint or target,
                timing['move_seconds'] if self.moved else self.approach_duration,
                timing['max_joint_speed'],timing['max_joint_acceleration'],self.active_max_jerk)
            self.wait_reason=('Inspection reposition toward '+target_name if phase=='candidate_inspection' and self.approach_waypoint is not None
                              else 'Raising near post before '+target_name if self.approach_waypoint is not None
                              else 'Moving to '+target_name)
            self.get_logger().info(f'{phase}: {self.wait_reason} ({self.active_duration:.2f}s trajectory)')
            if phase=='candidate_inspection':
                self.inspection_trajectory={
                    'viewpoint':target_name,'profile':'seventh_order' if self.active_max_jerk is not None else 'cubic',
                    'initial_joints_rad':list(self.initial),'target_joints_rad':list(target),
                    'leg_target_joints_rad':list(self.approach_waypoint or target),
                    'duration_seconds':self.active_duration,'max_joint_speed':timing['max_joint_speed'],
                    'max_joint_acceleration':timing['max_joint_acceleration'],'max_joint_jerk':self.active_max_jerk,
                    'waypoints':route}
                changes=[round(math.degrees(b-a),1) for a,b in zip(self.initial[:6],target[:6])]
                self.get_logger().info(f'Inspection joint changes (deg): {changes}; jerk limit={self.active_max_jerk}')
        elapsed=self.motion_time()-self.started
        if self.approach_waypoint is not None:
            target=self.approach_waypoint
        command=smooth_target(self.initial,target,elapsed,self.active_duration,
                              jerk_limited=self.active_max_jerk is not None)
        self.last_motion_command=list(command)
        self.command(command)
        error=max(abs(a-b) for a,b in zip(self.actual[:6],target[:6]))
        gripper_error=abs(self.actual[6]-target[6])
        if elapsed>=self.active_duration and error<.025 and gripper_error<.002:
            self.wait_reason='Settling at '+target_name
            if self.settled_since is None:self.settled_since=self.motion_time()
            if (self.approach_waypoint is not None
                    and self.motion_time()-self.settled_since>=self.active_settle_seconds):
                if self.remaining_waypoints:
                    target=self.remaining_waypoints.pop(0)
                    self.approach_waypoint=list(target)
                else:
                    target=self.targets[target_name] if ahead else self.select_target(target_name)
                    self.approach_waypoint=None
                self.initial=list(self.last_motion_command)
                self.started=self.motion_time();self.settled_since=None
                timing=self.motion_settings(self.active_phase)
                self.active_max_jerk=timing['max_joint_jerk']
                self.active_duration=move_duration(self.initial,target,timing['move_seconds'],
                    timing['max_joint_speed'],timing['max_joint_acceleration'],self.active_max_jerk)
                if self.active_phase=='candidate_inspection' and self.inspection_trajectory is not None:
                    self.inspection_trajectory.update(initial_joints_rad=list(self.initial),
                        leg_target_joints_rad=list(target),duration_seconds=self.active_duration)
                self.wait_reason=('Inspection reposition leg for '+target_name if self.active_phase=='candidate_inspection'
                                  else 'Raised near post; rotating to '+target_name)
                self.get_logger().info(self.wait_reason)
                return
            ready=ahead is not None or (self.view_client.service_is_ready() if view_request is not None else self.client.service_is_ready())
            if self.motion_time()-self.settled_since>=self.active_settle_seconds and ready:
                if ahead:
                    if self.discovery.get('next_survey_pose')!=ahead:
                        self.clear_motion()
                    self.wait_reason='Waiting for segmentation before capture at '+target_name
                    return
                self.request_key=key
                self.capture_future=(self.report_view(view_request, 'arrived') if view_request is not None
                                     else self.client.call_async(Trigger.Request()))
                self.moved=True
                self.wait_reason='Capturing '+target_name
                self.get_logger().info('Capturing '+target_name)
                return  # Arrival succeeded; do not also apply the settle deadline.
        else:self.settled_since=None
        if elapsed>self.active_duration+10.:
            errors=[a-b for a,b in zip(self.actual[:6],target[:6])]
            worst=max(range(6),key=lambda i:abs(errors[i]))
            ready=self.view_client.service_is_ready() if view_request is not None else self.client.service_is_ready()
            detail=(f'joint{worst+1} error={math.degrees(errors[worst]):+.2f} deg '
                    f'(actual={self.actual[worst]:.4f}, target={target[worst]:.4f} rad; '
                    'limit 1.43 deg); '
                    f'gripper error={gripper_error*1000:.2f} mm '
                    f'(actual={self.actual[6]:.4f}, target={target[6]:.4f} m; limit 2 mm); '
                    f'capture service ready={ready}')
            cause=('Capture service unavailable at ' if error<.025 and gripper_error<.002 and not ready
                   else 'Arm failed to settle at ')
            self.fault_stop(cause+target_name+': '+detail)

    def publish_status(self):
        reason=self.fault or self.wait_reason
        if reason != self.logged_wait:
            self.get_logger().info(reason);self.logged_wait=reason
        status={'reason':reason,'mode':'piper_joint_position','viewpoint':self.active,'fault':self.fault,
                'inspection_focus':self.inspection_focus,
                'inspection_trajectory':self.inspection_trajectory,
                'state':'fault' if self.fault else 'moving_or_capturing' if self.active else 'waiting'}
        request=(self.discovery or {}).get('view_request') or {}
        status['phase']=request.get('phase')
        focus=request.get('focus') or {}
        status['candidate_id']=focus.get('candidate_id',focus.get('id'))
        self.status_pub.publish(String(data=json.dumps(status)))


def main(args=None):
    from .runtime import run
    run(ScanNode,args)
