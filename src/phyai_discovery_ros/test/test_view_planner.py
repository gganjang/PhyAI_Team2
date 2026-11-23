"""Configured planning consumes candidate requests and excludes free objects."""
import json
from pathlib import Path

import pytest

from phyai_discovery_ros.view_planner import create_view_planner
from phyai_discovery_ros.sim_config import synchronize

ROOT=Path(__file__).resolve().parents[3]
ROBOT=Path('/opt/mujoco_menagerie/agilex_piper/piper.xml')


def test_unconfigured_planner_keeps_hardware_backend_optional():
    with pytest.raises(NotImplementedError):create_view_planner({}, {})


@pytest.mark.parametrize('blocked,expected_cost',[(False,3.),(True,5.)])
def test_coarse_survey_order_minimises_travel_using_only_clear_edges(blocked,expected_cost):
    from types import SimpleNamespace
    from phyai_discovery_ros.hardware_config import order_survey
    targets=[[x,0.,0.,0.,0.,0.,.035] for x in (3.,1.,2.)]
    initial=[0.,0.,0.,0.,0.,0.,.035]
    clear=lambda a,b:not (blocked and {a[0],b[0]}=={2.,3.})
    ordered=order_survey(SimpleNamespace(_clear_path=clear),targets,initial)
    assert sorted(q[0] for q in ordered)==[1.,2.,3.]
    previous=initial;cost=0.
    for target in ordered:
        assert clear(previous,target)
        cost+=abs(previous[0]-target[0]);previous=target
    assert cost==expected_cost


def hardware_config(name='discovery.json'):
    pytest.importorskip('mujoco')
    if not ROBOT.exists():pytest.skip('Menagerie model required')
    from phyai_discovery_ros.view_planner import resolve_planner_paths
    path=ROOT/'src/phyai_discovery_ros/config'/name
    config=json.loads(path.read_text())
    resolve_planner_paths(config,path.parent)
    return config


def test_large_survey_order_shortens_route_without_adding_blocked_edges():
    from types import SimpleNamespace
    from phyai_discovery_ros.hardware_config import order_survey
    values=[0.,16.,1.,15.,2.,14.,3.,13.,4.,12.,5.,11.,6.,10.,7.,9.,8.]
    targets=[[x,0.,0.,0.,0.,0.,.035] for x in values]
    initial=[-1.,0.,0.,0.,0.,0.,.035]
    clear=lambda a,b:{a[0],b[0]}!={0.,1.}
    result=order_survey(SimpleNamespace(_clear_path=clear),targets,initial)
    assert sorted(q[0] for q in result)==sorted(values)
    before=sum(abs(a-b) for a,b in zip([-1.]+values,values))
    after=sum(abs(a[0]-b[0]) for a,b in zip([initial]+result,result))
    assert after<before/2
    assert all(clear(a,b) for a,b in zip([initial]+result,result))


def test_compact_sweep_rejects_blocked_rotation():
    from types import SimpleNamespace
    from phyai_discovery_ros.hardware_config import order_rotation_sweep
    targets=[[x,.5,-1.3,0.,1.22,0.,.035] for x in (1.,-2.,2.,0.)]
    initial=[-.5,.5,-1.3,0.,1.22,0.,.035]
    with pytest.raises(ValueError,match='No clear monotonic rotation'):
        order_rotation_sweep(SimpleNamespace(_clear_path=lambda *_:False),targets,initial)


def test_cobra_sweep_keeps_posture_and_inserts_small_signed_base_steps():
    import numpy as np
    from types import SimpleNamespace
    from phyai_discovery_ros.hardware_config import order_rotation_sweep
    posture=[.5,-1.15,0.,1.22,0.,.035]
    targets=[[angle,*posture] for angle in np.deg2rad([-150,0,150])]
    path=order_rotation_sweep(SimpleNamespace(_clear_path=lambda *_:True),targets,
                             [np.deg2rad(140),*posture],np.deg2rad(20))
    yaw=np.array([q[0] for q in path])
    assert np.all(np.diff(yaw)<0)
    assert np.max(np.abs(np.diff(yaw)))<=np.deg2rad(20)+1e-9
    assert np.max(np.abs(yaw))<np.pi
    np.testing.assert_allclose(np.array(path)[:,1:],np.tile(posture,(len(path),1)))


def test_hardware_camera_matches_host_tf_and_rotates_with_joint6():
    import mujoco
    import numpy as np
    from phyai_discovery_ros.view_planner import installation_model
    config=hardware_config()
    model=installation_model(config);data=mujoco.MjData(model)
    measured=config['view_planner']['installation']['validation']
    addresses=[model.joint(f'joint{i}').qposadr[0] for i in range(1,7)]
    data.qpos[addresses]=measured['joints'][:6]
    mujoco.mj_forward(model,data)
    camera=data.camera('wrist');expected=np.asarray(measured['wrist_camera_to_base'])
    np.testing.assert_allclose(camera.xpos,expected[:3,3],atol=1e-6)
    np.testing.assert_allclose(camera.xmat.reshape(3,3)@np.diag([1.,-1.,-1.]),expected[:3,:3],atol=1e-6)
    before=camera.xmat.copy()
    data.qpos[addresses[5]]+=.3
    mujoco.mj_forward(model,data)
    assert not np.allclose(camera.xmat,before)
    assert not any(model.jnt_type==mujoco.mjtJoint.mjJNT_FREE)


def test_hardware_survey_has_broad_coverage_clear_paths_and_focused_views():
    import numpy as np
    from phyai_discovery.geometry import inside_polygon
    from phyai_discovery_ros.hardware_config import table_regions, use_tabletop_frame
    from piper_mujoco_sim.survey import table_image_fractions
    config=hardware_config()
    # Pool size does not change saved survey path validation.
    config['view_planner']['sample_count']=3000
    planner=create_view_planner(config,config['scan']['targets'])
    scene=json.loads(Path(config['view_planner']['scene_config']).read_text())
    regions,rotation=table_regions(scene)
    regions['survey_coverage']={}
    use_tabletop_frame(regions,scene,rotation)
    np.testing.assert_allclose(config['scope'],regions['scope'])
    np.testing.assert_allclose(config['table_polygon'],regions['table_polygon'])
    assert config['table_plates']==regions['table_plates']
    coverage=config['survey_coverage']
    # Official PiPER-L collision geometry changes visible regions. Partial
    # surveys must retain blind samples and truthfully report their status.
    assert all(0<=rate<=1 for rate in coverage['plate_coverage'])
    assert all(0<=rate<=1 for rate in coverage['broad_plate_coverage'])
    assert coverage['meets_minimum_plate_coverage']==(min(coverage['plate_coverage'])>=.95)
    assert len(coverage['uncovered_points'])<coverage['sample_count']
    assert all(inside_polygon(point[:2],config['table_polygon']) for point in coverage['uncovered_points'])
    assert config['survey_coverage']['rgb_mask_checked']
    assert config['survey_coverage']['excluded_radius'] is None
    assert config['excluded_regions']==[]
    compact=config['view_planner']['survey_motion']
    assert config['survey_coverage']['proposal_only']
    assert config['survey_coverage']['overhead_samples']>0
    stages=config['survey_stages']
    assert len(stages['broad'])<=compact.get('max_views',32)
    assert stages['near_post']==[]
    assert not config['view_planner']['near_post_survey']['enabled']
    assert config['viewpoints']==stages['broad']+stages['near_post']+stages['coverage']
    assert config['scan']['motion_profiles']['coverage']==config['scan']['motion_profiles']['candidate_inspection']
    assert config['discovery_mode']=='full'
    azimuths=np.array([config['scan']['targets'][label][0] for label in stages['broad']])
    assert np.all(np.diff(azimuths)>=0) or np.all(np.diff(azimuths)<=0)
    assert np.ptp([q[0] for q in config['scan']['targets'].values()])>np.deg2rad(250)
    broad=np.array([config['scan']['targets'][label] for label in stages['broad']])
    assert broad[0,2]==pytest.approx(-1.1+np.deg2rad(5))
    before=broad[0].copy();before[2]-=np.deg2rad(5)
    _,before_axes=planner.pose(before)
    _,after_axes=planner.pose(broad[0])
    pitch_change=np.degrees(np.arccos(before_axes[2,2])-np.arccos(after_axes[2,2]))
    assert pitch_change==pytest.approx(5.,abs=.01)
    np.testing.assert_allclose(broad[:,1:],np.tile(broad[0,1:],(len(broad),1)))
    assert np.max(np.abs(np.diff(broad[:,0])))<=compact['max_base_step']+1e-9
    assert np.max(np.abs(broad[:,0]))<=config['scan']['cable_yaw_limit']<=np.pi
    assert config['scan']['raise_before_sweep']
    current=config['view_planner']['installation']['reference_joints']
    raised=[current[0],*broad[0,1:].tolist()]
    assert planner._clear_path(current,raised)
    assert planner._clear_path(raised,broad[0].tolist())
    positions=[];rotations=[]
    for label in config['viewpoints']:
        target=config['scan']['targets'][label]
        assert planner._clear_path(current,target)
        assert all(low<=angle<=high for angle,(low,high) in zip(target[:6],config['scan']['joint_limits']))
        assert target[6]==.035
        eye,axes=planner.pose(target);positions.append(eye);rotations.append(axes)
        stage=config['view_planner']['near_post_survey'] if label in stages['near_post'] else compact
        low,high=stage['camera_height_range']
        assert low<=eye[2]<=high
        import mujoco
        mujoco.mj_forward(planner.model,planner.data)
        gripper=(planner.data.body('link7').xpos+planner.data.body('link8').xpos)/2
        assert np.linalg.norm(gripper[:2])<=stage['max_gripper_radius']
        forward=-axes[:,2]
        centre=eye+(.015-eye[2])/forward[2]*forward
        from phyai_discovery.coordinates import map_origin
        if label not in stages['coverage']:
            assert inside_polygon((centre-map_origin(config))[:2],config['table_polygon'])
        current=target
    scene=json.loads(Path(config['view_planner']['scene_config']).read_text())
    _,rotation=table_regions(scene)
    fractions=table_image_fractions(np.asarray(positions),np.asarray(rotations),scene,
        [*scene['post_position'],0.],rotation,config['view_planner']['installation']['cameras']['wrist'])
    assert min(fractions)>=config['survey_coverage']['minimum_table_image_fraction']
    # Inspect a candidate in the final broad view without requiring a large
    # camera flip or a near-post transit that is no longer part of the scan.
    current=config['scan']['targets'][stages['broad'][-1]]
    eye,axes=planner.pose(current);forward=-axes[:,2]
    point=eye+(.0075-eye[2])/forward[2]*forward
    focus={'id':'measured_probe','position':point.tolist(),
           'size':[.075,.025,.015],'priority':3,'supporting_views':[]}
    plan=planner.choose([focus],current,'refine_1')
    assert plan is not None
    assert np.max(np.abs(np.asarray(plan['joints'][3:6])-current[3:6]))<=config['view_planner']['inspection_motion']['max_wrist_joint_change']
    _,before=planner.pose(current);_,after=planner.pose(plan['joints'])
    assert np.arccos(np.clip((np.trace(before.T@after)-1)/2,-1,1))<=config['view_planner']['inspection_motion']['max_camera_rotation']
    assert planner._clear_path(current,plan['joints'])
    assert planner._outside_robot_mask(plan['joints'],focus['position'],.045)


def test_hardware_survey_rejects_stale_joint_kinematics():
    from phyai_discovery_ros.view_planner import installation_model
    config=hardware_config()
    config['view_planner']['installation']['joint_frames']['joint3']['xyz'][0]+=.01
    with pytest.raises(ValueError,match='Installation model changed'):
        installation_model(config)


def test_mounting_yaw_rotates_table_but_preserves_host_camera_kinematics(tmp_path):
    import mujoco
    import numpy as np
    from phyai_discovery_ros.view_planner import installation_xml,installation_model
    from phyai_discovery_ros.hardware_config import table_regions
    config=hardware_config()
    scene=json.loads(Path(config['view_planner']['scene_config']).read_text())
    scene['robot_yaw_degrees']=90.
    path=tmp_path/'rotated_scene.json';path.write_text(json.dumps(scene))
    config['view_planner']['scene_config']=str(path)
    model=mujoco.MjModel.from_xml_string(installation_xml(config))
    data=mujoco.MjData(model)
    measured=config['view_planner']['installation']['validation']
    addresses=[model.joint(f'joint{i}').qposadr[0] for i in range(1,7)]
    data.qpos[addresses]=measured['joints'][:6];mujoco.mj_forward(model,data)
    expected=np.asarray(measured['wrist_camera_to_base'])
    np.testing.assert_allclose(data.camera('wrist').xpos,expected[:3,3],atol=1e-6)
    np.testing.assert_allclose(data.camera('wrist').xmat.reshape(3,3)@np.diag([1.,-1.,-1.]),expected[:3,:3],atol=1e-6)
    regions,rotation=table_regions(scene)
    for plate in regions['table_plates']:
        row,column=divmod(plate['id']-1,2)
        np.testing.assert_allclose(np.mean(plate['polygon'],axis=0),
                                   data.geom(f'table_{row}_{column}').xpos[:2],atol=1e-8)
    np.testing.assert_allclose(rotation@[1.,0.,0.],[0.,-1.,0.],atol=1e-8)
    with pytest.raises(ValueError,match='Installation model changed'):
        installation_model(config)


def test_configured_model_plans_measured_focus_and_ignores_free_geometry():
    mujoco=pytest.importorskip('mujoco')
    if not ROBOT.exists():pytest.skip('Menagerie model required')
    import numpy as np
    template=json.loads((ROOT/'src/phyai_discovery_ros/config/mujoco_interface.json').read_text())
    survey=json.loads((ROOT/'apps/junh/config/mujoco.json').read_text())
    config=synchronize(template,survey)
    config['view_planner']={'backend':'mujoco_geometry','robot_path':str(ROBOT),
        'scene_config':str(ROOT/'sim/piper_mujoco_sim/config/lab_scene.json')}
    planner=create_view_planner(config,config['scan']['targets'])
    model=planner.model
    for joint in range(model.njnt):
        if model.jnt_type[joint]==mujoco.mjtJoint.mjJNT_FREE:
            ids=np.flatnonzero(model.geom_bodyid==model.jnt_bodyid[joint])
            assert np.all(model.geom_contype[ids]==0)
            assert np.all(model.geom_conaffinity[ids]==0)
            assert np.all(model.geom_group[ids]==5)
    current=config['scan']['targets'][config['viewpoints'][-1]]
    focus={'id':'measured','position':[.12,.47,.0075],
        'size':[.075,.025,.015],'priority':3,'supporting_views':[]}
    plan=planner.choose([focus],current,'focus_1')
    assert plan is not None and plan['focus']=='measured'
    assert planner._outside_robot_mask(plan['joints'],focus['position'],.045)


def test_robot_exclusion_uses_capture_matched_feedback_and_not_free_objects():
    mujoco=pytest.importorskip('mujoco')
    if not ROBOT.exists():pytest.skip('Menagerie model required')
    import numpy as np
    from types import SimpleNamespace
    from phyai_discovery_ros.view_planner import RobotMaskModel
    from phyai_discovery.observations import Intrinsics
    template=json.loads((ROOT/'src/phyai_discovery_ros/config/mujoco_interface.json').read_text())
    survey=json.loads((ROOT/'apps/junh/config/mujoco.json').read_text())
    config=synchronize(template,survey)
    config['view_planner']={'backend':'mujoco_geometry','robot_path':str(ROBOT),
        'scene_config':str(ROOT/'sim/piper_mujoco_sim/config/lab_scene.json')}
    model=RobotMaskModel(config)
    q=config['scan']['targets'][config['viewpoints'][0]]
    model.data.qpos[model.addresses]=q[:6]
    mujoco.mj_forward(model.model,model.data)
    cam=model.data.camera('overhead')
    transform=np.eye(4)
    transform[:3,:3]=cam.xmat.reshape(3,3)@np.diag([1.,-1.,-1.])
    transform[:3,3]=cam.xpos
    capture=SimpleNamespace(stamp=100.,camera_to_map=transform,
        intrinsics=Intrinsics(640,480,580.,580.,319.5,239.5))
    first=model.project(capture,[(100.,q)])
    assert first.any()
    joint=model.model.joint('block_1_joint')
    model.data.qpos[joint.qposadr[0]:joint.qposadr[0]+3]=[10,20,30]
    np.testing.assert_array_equal(model.project(capture,[(100.,q)]),first)
    with pytest.raises(ValueError,match='capture-matched'):
        model.project(capture,[(99.,q)])


def test_tabletop_translation_preserves_hardware_robot_mask_pixels():
    import mujoco
    import numpy as np
    from phyai_discovery import Capture, Intrinsics
    from phyai_discovery.coordinates import camera_to_map
    from phyai_discovery_ros.view_planner import RobotMaskModel
    from piper_mujoco_sim.robot_mask import robot_mask
    config=hardware_config()
    model=RobotMaskModel(config)
    q=config['view_planner']['installation']['reference_joints']
    model.data.qpos[model.addresses]=q[:6]
    model.data.qpos[model.model.joint('joint7').qposadr[0]]=q[6]
    model.data.qpos[model.model.joint('joint8').qposadr[0]]=-q[6]
    mujoco.mj_forward(model.model,model.data)
    cam=model.data.camera('overhead')
    transform=np.eye(4)
    transform[:3,:3]=cam.xmat.reshape(3,3)@np.diag([1.,-1.,-1.])
    transform[:3,3]=cam.xpos
    k=Intrinsics(640,480,580.,580.,319.5,239.5)
    base_capture=Capture('overhead',100.,'optical','base_link','test',k,None,transform,'overview')
    expected=robot_mask(model.model,model.data,base_capture)
    assert expected.any()
    from dataclasses import replace
    mapped=replace(base_capture,map_frame='tabletop',camera_to_map=camera_to_map(config,transform))
    np.testing.assert_array_equal(model.project(mapped,[(100.,q)]),expected)

    simulator_model=RobotMaskModel(config,use_scene_base=True)
    base_in_world=np.array([-.0655,.0655,0.])
    camera_to_world=transform.copy()
    camera_to_world[:3,3]+=base_in_world
    world_capture=replace(base_capture,map_frame='world',camera_to_map=camera_to_world)
    with pytest.raises(ValueError,match='simulator robot base position'):
        simulator_model.project(world_capture,[(100.,q)])
    np.testing.assert_array_equal(
        simulator_model.project(world_capture,[(100.,q)],base_in_map=base_in_world),expected)


@pytest.mark.parametrize('use_scene_base',[False,True])
def test_overhead_coverage_skip_keeps_sweep_endpoints_and_unknown_cells(use_scene_base):
    import numpy as np
    from phyai_discovery import Intrinsics
    from phyai_discovery.coordinates import map_origin
    from phyai_discovery.workspace import Workspace
    from phyai_discovery_ros.view_planner import RobotMaskModel
    config=hardware_config()
    model=RobotMaskModel(config,use_scene_base=use_scene_base)
    broad=['sweep_5','sweep_7','sweep_9']
    targets=config['scan']['targets']
    workspace=Workspace(scope=(-.84,.84,-.5,.5))
    workspace.coverage={(x,y):{'state':'observed','last_observed':1.,'camera':'overhead','explored':True}
                        for x in range(-42,42) for y in range(-25,25)}
    intrinsics=Intrinsics(640,480,620.,620.,320.,240.)
    base=np.array([-.0655,.0655,0.]) if use_scene_base else -map_origin(config)
    assert model.covered_survey_views(workspace,intrinsics,broad,targets,base,[.075,.025,.015])==['sweep_7']

    camera=model.data.camera('wrist')
    ray=-camera.xmat.reshape(3,3)[:,2]
    centre=camera.xpos+base-ray*camera.xpos[2]/ray[2]
    cell=tuple(np.floor(centre[:2]/workspace.cell_size).astype(int))
    del workspace.coverage[cell]
    assert model.covered_survey_views(workspace,intrinsics,broad,targets,base,[.075,.025,.015])==[]


def test_hardware_reframing_preserves_piper_l_inertia_and_host_gripper_parents():
    import mujoco
    import numpy as np
    from phyai_discovery_ros.view_planner import installation_xml
    config=hardware_config()
    # Compare physical inertia in a common base frame, independently of the
    # principal-axis representation selected by MuJoCo's compiler.
    source=mujoco.MjModel.from_xml_path(str(ROBOT))
    hardware=mujoco.MjModel.from_xml_string(installation_xml(config))
    states=[]
    for model in (source,hardware):
        data=mujoco.MjData(model)
        mujoco.mj_forward(model,data)
        states.append(data)
    for i in range(1,9):
        name=f'link{i}'
        tensors=[]
        centres=[]
        masses=[]
        for model,data in zip((source,hardware),states):
            body=model.body(name)
            orientation=data.ximat[body.id].reshape(3,3)
            tensors.append(orientation@np.diag(body.inertia)@orientation.T)
            centres.append(data.xipos[body.id])
            masses.append(body.mass[0])
        np.testing.assert_allclose(tensors[1],tensors[0],atol=1e-10)
        np.testing.assert_allclose(centres[1],centres[0],atol=1e-10)
        assert masses[1]==pytest.approx(masses[0])
    for i in (7,8):
        body=hardware.body(f'link{i}')
        assert hardware.body(int(hardware.body_parentid[body.id])).name=='link6'
        np.testing.assert_allclose(body.pos,config['view_planner']['installation']['joint_frames'][f'joint{i}']['xyz'])


def test_temporary_host_limits_apply_to_survey_and_focused_planning():
    import numpy as np
    full=hardware_config()
    temporary=hardware_config('discovery_host_legacy_limits.json')
    assert temporary['view_planner']['installation']==full['view_planner']['installation']
    assert temporary['cameras']==full['cameras']
    limits=np.asarray(temporary['scan']['joint_limits'])
    original=np.asarray(full['scan']['joint_limits'])
    assert np.all(limits[:,0]>=original[:,0])
    assert np.all(limits[:,1]<=original[:,1])
    np.testing.assert_allclose(limits[4],[-1.22,1.22])
    # Default hardware operation must respect the same observed host clamp.
    np.testing.assert_allclose(original[4],[-1.22,1.22])
    temporary['view_planner']['sample_count']=3000
    planner=create_view_planner(temporary,temporary['scan']['targets'])
    assert len(planner.pool)>0
    assert np.all(planner.pool[:,:6]>=limits[:,0])
    assert np.all(planner.pool[:,:6]<=limits[:,1])
    for i in range(1,7):
        np.testing.assert_allclose(planner.model.joint(f'joint{i}').range,limits[i-1])
    previous=temporary['view_planner']['installation']['reference_joints']
    for label in temporary['viewpoints']:
        target=temporary['scan']['targets'][label]
        assert np.all(np.asarray(target[:6])>=limits[:,0]),label
        assert np.all(np.asarray(target[:6])<=limits[:,1]),label
        assert planner._clear_path(previous,target),label
        previous=target
    coverage=temporary['survey_coverage']
    assert coverage['meets_minimum_plate_coverage']==(min(coverage['plate_coverage'])>=.95)
