"""Optional view planning from a configured robot/camera installation model.

Only requested by configurations selecting this backend. Hardware installations
without MuJoCo continue to use fixed poses or their own ScanNode adapter.
"""
from pathlib import Path
import xml.etree.ElementTree as ET


def resolve_planner_paths(config, directory):
    options=config.get('view_planner',{})
    for key in ('robot_path','scene_config'):
        if key in options:
            options[key]=str((Path(directory)/options[key]).resolve())


def installation_xml(config):
    """Reuse the lab scene with optional measured hardware camera attachments."""
    import mujoco
    import numpy as np
    from piper_mujoco_sim.scene import build_scene
    options=config['view_planner']
    xml=build_scene(Path(options['robot_path']),Path(options['scene_config']))
    hardware=options.get('installation')
    if hardware is None:
        return xml
    from phyai_discovery.coordinates import source_frame
    if source_frame(config)!='base_link':
        raise ValueError('The hardware installation model requires base_link source coordinates.')
    root=ET.fromstring(xml)
    world=root.find('worldbody')
    base=world.find("body[@name='base_link']")
    # Full scene-to-base transform: mounting yaw affects table plates as well
    # as translation. Calibrated camera mounts remain in host link frames.
    base_rotation=np.empty(9)
    mujoco.mju_quat2Mat(base_rotation,np.fromstring(base.get('quat','1 0 0 0'),sep=' '))
    scene_to_base=base_rotation.reshape(3,3).T
    offset=np.fromstring(base.get('pos','0 0 0'),sep=' ')
    vector=lambda v:' '.join(str(float(x)) for x in v)
    for element in world:
        if element.tag in ('body','geom','light'):
            element.set('pos',vector(scene_to_base@(np.fromstring(element.get('pos','0 0 0'),sep=' ')-offset)))
            if element.tag in ('body','geom'):
                old=np.empty(9);mujoco.mju_quat2Mat(old,np.fromstring(element.get('quat','1 0 0 0'),sep=' '))
                quat=np.empty(4);mujoco.mju_mat2Quat(quat,(scene_to_base@old.reshape(3,3)).ravel())
                element.set('quat',vector(quat))
    # The host URDF uses different link frames from Menagerie. Reframe the
    # existing collision/visual meshes while adopting the measured joint chain.
    frames=hardware.get('joint_frames',{})
    if frames:
        import math
        nominal=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'))
        nominal_data=mujoco.MjData(nominal);mujoco.mj_forward(nominal,nominal_data)
        poses={'base_link':np.eye(4),'gripper_base':None}
        for i in range(1,9):
            frame=frames[f'joint{i}']
            body=root.find(f".//body[@name='link{i}']")
            roll,pitch,yaw=frame['rpy']
            cr,sr,cp,sp,cy,sy=math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch),math.cos(yaw),math.sin(yaw)
            rotation=np.array([[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],
                [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]])
            relative=np.eye(4);relative[:3,:3]=rotation;relative[:3,3]=frame['xyz']
            parent=frame['parent']
            if parent=='gripper_base':parent='link6'  # Host fixed joint is identity.
            # PiPER-L includes fixed flange/gripper bodies. The saved host frame
            # is relative to its declared parent, not those intermediate bodies.
            old_parent=next(element for element in root.iter('body') if body in list(element))
            new_parent=root.find(f".//body[@name='{parent}']")
            if old_parent is not new_parent:
                old_parent.remove(body)
                new_parent.append(body)
            pose=poses[parent]@relative;poses[f'link{i}']=pose
            original=np.eye(4)
            original[:3,:3]=nominal_data.body(f'link{i}').xmat.reshape(3,3)
            original[:3,3]=nominal_data.body(f'link{i}').xpos
            correction=np.linalg.inv(pose)@original
            quat=np.empty(4);mujoco.mju_mat2Quat(quat,rotation.ravel())
            body.set('pos',vector(relative[:3,3]));body.set('quat',vector(quat))
            for geometry in body.findall('geom'):
                position=np.fromstring(geometry.get('pos','0 0 0'),sep=' ')
                old_quat=np.fromstring(geometry.get('quat','1 0 0 0'),sep=' ')
                old_rotation=np.empty(9);mujoco.mju_quat2Mat(old_rotation,old_quat)
                mujoco.mju_mat2Quat(quat,(correction[:3,:3]@old_rotation.reshape(3,3)).ravel())
                geometry.set('pos',vector(correction[:3,:3]@position+correction[:3,3]))
                geometry.set('quat',vector(quat))
            inertial=body.find('inertial')
            if inertial is not None:
                # Compile either full or diagonal source tensors into principal
                # axes first. MJCF forbids fullinertia together with a quat.
                source=nominal.body(f'link{i}')
                principal=np.empty(9)
                mujoco.mju_quat2Mat(principal,source.iquat)
                mujoco.mju_mat2Quat(quat,(correction[:3,:3]@principal.reshape(3,3)).ravel())
                inertial.attrib.clear()
                inertial.set('mass',str(float(source.mass[0])))
                inertial.set('pos',vector(correction[:3,:3]@source.ipos+correction[:3,3]))
                inertial.set('diaginertia',vector(source.inertia))
                inertial.set('quat',vector(quat))
            body.find(f"joint[@name='joint{i}']").set('axis',vector(frame['axis']))
    for i,limits in enumerate(config['scan']['joint_limits'],1):
        root.find(f".//joint[@name='joint{i}']").set('range',vector(limits))
    for name in ('overhead','wrist'):
        camera=hardware['cameras'][name]
        if camera['parent']!=('base_link' if name=='overhead' else 'link6'):
            raise ValueError('Hardware camera parent must be base_link for overhead and link6 for wrist.')
        transform=np.asarray(camera['optical_to_parent'],dtype=float)
        if (transform.shape!=(4,4) or not np.isfinite(transform).all()
                or not np.allclose(transform[3],[0,0,0,1])
                or not np.allclose(transform[:3,:3].T@transform[:3,:3],np.eye(3),atol=1e-6)
                or not np.isclose(np.linalg.det(transform[:3,:3]),1.)):
            raise ValueError('Hardware camera calibration requires a rigid optical_to_parent transform.')
        mount=root.find(".//body[@name='overhead_mount']") if name=='overhead' else root.find(".//body[@name='wrist_camera_mount']")
        if name=='wrist':
            parent=root.find(".//body[@name='link6']")
            old_parent=next(e for e in root.iter('body') if mount in list(e))
            old_parent.remove(mount)
            parent.append(mount)
            body=mount.find("body[@name='wrist_camera_body']")
            body.set('pos','0 0 0');body.set('quat','1 0 0 0')
            # The generic mount plate must remain behind the calibrated lens.
            mount.find("geom[@name='wrist_camera_mount_plate']").set('pos','0 0 .04')
        rotation=transform[:3,:3]@np.diag([1.,-1.,-1.])
        quat=np.empty(4);mujoco.mju_mat2Quat(quat,rotation.ravel())
        mount.set('pos',vector(transform[:3,3]));mount.set('quat',vector(quat))
        lens=root.find(f".//camera[@name='{name}']")
        lens.set('quat','1 0 0 0')
        import math
        lens.set('fovy',str(math.degrees(2*math.atan(camera['height']/(2*camera['fy'])))))
    # Blocks are unknown on hardware; do not retain simulator free objects at all.
    for body in list(world.findall('body')):
        if body.find('freejoint') is not None:world.remove(body)
    keyframe=root.find('keyframe')
    if keyframe is not None:root.remove(keyframe)
    return ET.tostring(root,encoding='unicode')


def installation_geometry(config,model):
    from piper_mujoco_sim.inspection import scan_geometry
    geometry=scan_geometry(model)
    if config['view_planner'].get('installation'):
        # A base-centred post alone cannot reveal a changed mounting yaw. Bind
        # saved poses to the actual tabletop geometry in the same base frame.
        geometry['table_plates']={f'table_{row}_{column}':{
            'position':model.geom(f'table_{row}_{column}').pos.tolist(),
            'quaternion':model.geom(f'table_{row}_{column}').quat.tolist(),
            'size':model.geom(f'table_{row}_{column}').size.tolist()}
            for row in range(2) for column in range(2)}
        geometry['joints']={f'joint{i}':{
            'range':model.joint(f'joint{i}').range.tolist(),
            'axis':model.joint(f'joint{i}').axis.tolist(),
            'position':model.body(f'link{i}').pos.tolist(),
            'quaternion':model.body(f'link{i}').quat.tolist()} for i in range(1,9)}
    return geometry


def installation_model(config):
    options=config.get('view_planner',{})
    if options.get('backend')!='mujoco_geometry':
        raise NotImplementedError('Candidate inspection requires a configured robot view planner.')
    import mujoco
    from .sim_config import geometry_matches
    model=mujoco.MjModel.from_xml_string(installation_xml(config))
    if not geometry_matches(config['survey_geometry'],installation_geometry(config,model)):
        raise ValueError('Installation model changed since survey generation; regenerate the survey and relaunch.')
    return model


def create_view_planner(config, targets):
    model=installation_model(config)
    from piper_mujoco_sim.inspection import InspectionPlanner
    # Free objects are disabled; focus positions come from measured requests.
    return InspectionPlanner(model,list(targets.values()),None,
        gripper=config['scan'].get('gripper_position',.035),rgb_mask_clearance=True,
        fixed_wrist_roll=not bool(config['view_planner'].get('installation')),
        allow_inverted=bool(config['view_planner'].get('installation')),
        sample_count=config['view_planner'].get('sample_count',3000),
        motion_preferences=config['view_planner'].get('inspection_motion'),
        allow_reposition=config['view_planner'].get('inspection_reposition',False))


class RobotMaskModel:
    """Project only known robot geometry at capture-matched measured joints."""
    def __init__(self, config):
        import mujoco
        self.config=config
        self.model=installation_model(config)
        self.data=mujoco.MjData(self.model)
        self.addresses=[self.model.joint(f'joint{i}').qposadr[0] for i in range(1,7)]

    def project(self, capture, history):
        import mujoco
        from piper_mujoco_sim.robot_mask import robot_mask
        if not history:
            raise ValueError('Waiting for capture-matched joint feedback for robot exclusion.')
        stamp,joints=min(history,key=lambda sample:abs(sample[0]-capture.stamp))
        if abs(stamp-capture.stamp)>.05:
            raise ValueError('Waiting for capture-matched joint feedback for robot exclusion.')
        self.data.qpos[self.addresses]=joints[:6]
        self.data.qpos[self.model.joint('joint7').qposadr[0]]=joints[6]
        self.data.qpos[self.model.joint('joint8').qposadr[0]]=-joints[6]
        mujoco.mj_kinematics(self.model,self.data)
        from dataclasses import replace
        from phyai_discovery.coordinates import camera_to_source, source_frame
        source_capture=(replace(capture,map_frame=source_frame(self.config),
            camera_to_map=camera_to_source(self.config,capture.camera_to_map))
            if self.config.get('coordinate_system') else capture)
        return robot_mask(self.model,self.data,source_capture)
