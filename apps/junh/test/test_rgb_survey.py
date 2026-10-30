"""Real SAM2 regression for discovery without the retired stacking application."""
import json
import os
from pathlib import Path

import numpy as np
import pytest

CHECKPOINT = (os.environ.get('PHYAI_DISCOVERY_TEST_SAM2_CHECKPOINT')
              or os.environ.get('JUNH_TEST_SAM2_CHECKPOINT'))
ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.skipif(not CHECKPOINT, reason='Set PHYAI_DISCOVERY_TEST_SAM2_CHECKPOINT for real SAM2')
@pytest.mark.parametrize('layout', ['saved_lab', 'beside_post'])
def test_saved_lab_brief_survey_then_candidate_inspection(tmp_path, layout):
    mujoco = pytest.importorskip('mujoco')
    from phyai_discovery import Capture, Discovery, Intrinsics, SearchState
    from piper_mujoco_sim.inspection import scan_geometry
    from junh_master.ros.sim_scan import SimScanNode
    from rclpy.parameter import Parameter
    import rclpy
    from piper_mujoco_sim.robot_mask import robot_mask
    from piper_mujoco_sim.scene import build_scene
    from piper_mujoco_sim.inspection import InspectionPlanner
    from dataclasses import replace
    robot = Path(os.environ.get('PIPER_TEST_ROBOT', '/opt/mujoco_menagerie/agilex_piper/piper.xml'))
    if not robot.is_file():
        pytest.skip('PIPER model required')
    config = json.loads((ROOT/'apps/junh/config/mujoco.json').read_text())
    config['segmentation']['checkpoint'] = CHECKPOINT
    scene_path = ROOT/'sim/piper_mujoco_sim/config/lab_scene.json'
    scene = json.loads(scene_path.read_text())
    if layout == 'beside_post':
        # A separate layout exposes the blind spot hidden by aggregate coverage.
        x, y = scene['post_position']
        scene['block_positions'] = [[x+dx, y+dy] for dx, dy in
            [(-.16, 0), (-.12, .12), (0, .16), (.12, .12),
             (.16, 0), (.04, -.16), (-.08, -.16)]]
        scene_path = tmp_path/'beside_post.json'
        scene_path.write_text(json.dumps(scene))
    model = mujoco.MjModel.from_xml_string(build_scene(str(robot), str(scene_path)))
    width, height = 1280, 960
    model.vis.global_.offwidth = max(model.vis.global_.offwidth, width)
    model.vis.global_.offheight = max(model.vis.global_.offheight, height)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, 0)
    for _ in range(500):
        mujoco.mj_step(model, data)
    # Test oracle only. Neither object count nor poses are passed to Discovery.
    expected_count = len(scene['block_positions'])
    truth = np.array([data.body(f'block_{i+1}').xpos.copy() for i in range(expected_count)])
    feature = Discovery(config)
    planner = InspectionPlanner(model, list(config['sim_scan']['targets'].values()))
    previous = data.qpos[:7].copy()
    path = tmp_path/'discovery.json'
    path.write_text(json.dumps(config))
    rclpy.init(domain_id=174)
    scan = SimScanNode(parameter_overrides=[Parameter('config_path', value=str(path)),
        Parameter('scene_config', value=str(scene_path))])
    scan.scene = {'scan_geometry': scan_geometry(model)}
    renderer = mujoco.Renderer(model, height=height, width=width)
    observations = []
    try:
        for index in range(100):  # Includes no-plan reports that reuse an inspection slot.
            if feature.search.state == SearchState.FINISHED:
                break
            label = feature.search.target
            camera = 'overhead' if feature.search.state == SearchState.OVERHEAD else 'wrist'
            if camera == 'wrist':
                scan.actual = data.qpos[:7].copy()
                scan.session=feature.workspace.session
                request=feature.view_request()
                scan.discovery={'view_request':request}
                joints = scan.select_target(label)
                if joints is None:
                    assert feature.report_view(request['request_id'],'unreachable',
                        session_id=request['session_id'],reason='No clear simulated view')
                    continue
            else:
                joints = previous
            assert planner._clear_path(previous, joints)
            previous = joints
            data.qpos[:7] = joints
            data.qpos[7] = -joints[6]
            mujoco.mj_forward(model, data)
            renderer.update_scene(data, camera)
            pixels = renderer.render().copy()
            cid = model.camera(camera).id
            transform = np.eye(4)
            transform[:3,:3] = data.cam_xmat[cid].reshape(3,3) @ np.diag([1.,-1.,-1.])
            transform[:3,3] = data.cam_xpos[cid]
            focal = height/(2*np.tan(np.deg2rad(model.cam_fovy[cid])/2))
            stamp = 10.*(index+1)
            frame = Capture(camera, stamp, camera+'_optical', 'world', 'rendered_rgb',
                Intrinsics(width,height,focal,focal,(width-1)/2,(height-1)/2), None, transform,
                label, rgb=pixels)
            frame = replace(frame, mask=robot_mask(model, data, frame))
            if camera == 'wrist':
                assert feature.report_view(request['request_id'],'arrived',
                    session_id=request['session_id'],now=stamp)
                feature.tick(stamp+.1, stationary=True)
            detection = feature.process(frame, stationary=True)
            observations.append({'camera': camera, 'viewpoint': label,
                'focus': request['focus']['candidate_id'] if camera == 'wrist' and request['focus'] else None,
                'candidates': [{'position': item.position, 'resolved': item.resolved,
                                'reasons': item.reasons} for item in detection.candidates]})
        (tmp_path/'observations.json').write_text(json.dumps(observations, indent=2))
        snapshot = feature.snapshot()
        (tmp_path/'discovery_snapshot.json').write_text(json.dumps(snapshot,indent=2))
        tracks = snapshot['workspace']['tracks']
        assert snapshot['search']['state'] == 'search_finished'
        assessment=snapshot['search']['assessment']
        assert all(block['meets_criteria'] for block in assessment['blocks']), assessment['reasons']
        if not snapshot['search']['result']['criteria_met']:
            assert snapshot['search']['result']['status']=='partial'
            assert (snapshot['workspace']['coverage']['unexplained_cells'] > 0
                    or assessment['explored_fraction'] < assessment['min_explored_fraction'])
        assert snapshot['search']['result']['pick_ready'] is False
        assert len(tracks) == expected_count
        assert not snapshot['workspace']['hypotheses']
        nearest = []
        for track in tracks:
            distances = np.linalg.norm(truth-track['position'], axis=1)
            nearest.append(int(distances.argmin()))
            assert distances.min() <= track['pose_uncertainty']['position_m']
            assert track['resolved']
            assert len({view for camera,view in track['supporting_views'] if camera=='wrist'}) >= 2
        assert len(set(nearest)) == expected_count
        assert not snapshot['workspace']['stacking_order']['assigned']
    finally:
        renderer.close()
        scan.destroy_node()
        rclpy.try_shutdown()
