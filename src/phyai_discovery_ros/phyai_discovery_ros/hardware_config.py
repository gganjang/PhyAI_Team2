"""Generate a hardware survey with the shared simulator geometry planner.

Reads only saved installation calibration and table geometry; never publishes ROS
commands and never uses simulator block positions to score views.
"""
import argparse
import copy
import json
from pathlib import Path

from .view_planner import installation_xml, installation_geometry, resolve_planner_paths


def order_survey(planner,targets,initial):
    """Reduce joint travel using clear edges; solve small surveys exactly."""
    import numpy as np
    count=len(targets)
    distance=lambda a,b:float(np.max(np.abs(np.asarray(a[:6])-b[:6])))
    edges={}
    for i in range(count):
        for j in range(i+1,count):
            if planner._clear_path(targets[i],targets[j]):
                edges[i,j]=edges[j,i]=distance(targets[i],targets[j])
    if count>14:
        # Avoid exponential DP for larger surveys. Reversing a path section
        # preserves its clear internal edges; check the new boundary edges.
        order=list(range(count))
        starts={i:distance(initial,q) for i,q in enumerate(targets)
                if planner._clear_path(initial,q)}
        def cost(path):
            return starts.get(path[0],float('inf'))+sum(
                edges.get((a,b),float('inf')) for a,b in zip(path,path[1:]))
        for _ in range(count*count):
            best=cost(order);replacement=None
            for begin in range(count-1):
                for end in range(begin+1,count):
                    candidate=order[:begin]+order[begin:end+1][::-1]+order[end+1:]
                    value=cost(candidate)
                    if value<best-1e-9:best=value;replacement=candidate
            if replacement is None:break
            order=replacement
        return [targets[i] for i in order]
    states={}
    for i,q in enumerate(targets):
        if planner._clear_path(initial,q):states[1<<i,i]=(distance(initial,q),None)
    for mask in range(1,1<<count):
        for last in range(count):
            state=states.get((mask,last))
            if state is None:continue
            for nxt in range(count):
                if mask&(1<<nxt) or (last,nxt) not in edges:continue
                key=(mask|(1<<nxt),nxt);cost=state[0]+edges[last,nxt]
                if key not in states or cost<states[key][0]:states[key]=(cost,last)
    final=(1<<count)-1
    endings=[(state[0],last) for (mask,last),state in states.items() if mask==final]
    if not endings:return targets
    _,last=min(endings);order=[];mask=final
    while last is not None:
        order.append(last);previous=states[mask,last][1];mask^=1<<last;last=previous
    return [targets[i] for i in reversed(order)]


def table_regions(scene):
    """Transform measured table corners from scene coordinates into base_link."""
    import math
    import numpy as np
    result={}
    yaw=-math.radians(scene['robot_yaw_degrees'])
    rotation=np.array([[math.cos(yaw),-math.sin(yaw),0.],
                       [math.sin(yaw),math.cos(yaw),0.],[0.,0.,1.]])
    def mapped(xy):
        return ((np.c_[xy,np.zeros(len(xy))]-[*scene['post_position'],0.])@rotation.T)[:,:2].tolist()
    width,depth=scene['table_size'];gap=scene['table_gap']
    hx,hy=width+gap/2,depth+gap/2
    result['table_polygon']=mapped([[-hx,-hy],[hx,-hy],[hx,hy],[-hx,hy]])
    bounds=np.asarray(result['table_polygon'])
    result['scope']=[float(bounds[:,0].min()),float(bounds[:,0].max()),float(bounds[:,1].min()),float(bounds[:,1].max())]
    result['table_plates']=[]
    for row in range(2):
        for column in range(2):
            x,y=(column-.5)*(width+gap),(row-.5)*(depth+gap)
            result['table_plates'].append({'id':row*2+column+1,'polygon':mapped(
                [[x-width/2,y-depth/2],[x+width/2,y-depth/2],
                 [x+width/2,y+depth/2],[x-width/2,y+depth/2]])})
    return result,rotation


def use_tabletop_frame(config, scene, rotation):
    """Convert base-frame survey regions to a map centred at the plate junction."""
    import numpy as np
    origin = rotation @ -np.array([*scene['post_position'], 0.])
    config['map_frame'] = 'tabletop'
    config['coordinate_system'] = {
        'source_frame': 'base_link', 'origin_in_source': origin.tolist(),
        'axes': 'parallel to base_link; metres; z=0 at tabletop'}
    config['scope'] = [v-origin[i//2] for i,v in enumerate(config['scope'])]
    for polygon in [config['table_polygon']] + [p['polygon'] for p in config['table_plates']]:
        for xy in polygon:
            xy[:] = (np.asarray(xy)-origin[:2]).tolist()
    for region in config.get('excluded_regions',[]):
        region['center']=(np.asarray(region['center'])-origin[:2]).tolist()
    def convert_report(report):
        if 'uncovered_points' in report:
            report['uncovered_points'] = [(np.asarray(p)-origin).tolist() for p in report['uncovered_points']]
        for value in report.values():
            if isinstance(value, dict):
                convert_report(value)
    convert_report(config['survey_coverage'])


def order_rotation_sweep(planner,targets,initial,max_base_step=None):
    """Visit azimuths in one direction rather than revisiting table sectors."""
    import numpy as np
    ordered=sorted(targets,key=lambda q:q[0])
    feasible=[]
    for path in (ordered,ordered[::-1]):
        pairs=list(zip([initial]+path,path))
        if all(planner._clear_path(a,b) for a,b in pairs):
            travel=sum(float(np.max(np.abs(np.asarray(a[:6])-b[:6]))) for a,b in pairs)
            feasible.append((travel,path))
    if not feasible:raise ValueError('No clear monotonic rotation connects the compact survey views.')
    path=min(feasible,key=lambda pair:pair[0])[1]
    if max_base_step is None:return path
    import math
    if not math.isfinite(max_base_step) or max_base_step<=0:
        raise ValueError('Maximum base step must be positive radians.')
    if any(not np.allclose(q[1:],path[0][1:],atol=1e-9) for q in path):
        raise ValueError('Cobra survey must retain one fixed raised posture.')
    dense=[path[0]]
    for target in path[1:]:
        previous=np.asarray(dense[-1])
        steps=math.ceil(abs(target[0]-previous[0])/max_base_step)
        for index in range(1,steps+1):
            q=(previous+(np.asarray(target)-previous)*index/steps).tolist()
            if not planner._clear_path(dense[-1],q):
                raise ValueError('Cobra rotation crosses a blocked path.')
            dense.append(q)
    return dense


def compact_rotation_pool(planner,options):
    """Elevated survey postures rotated around joint1, with bounded reach."""
    import itertools
    import mujoco
    import numpy as np
    grid=options['posture_grid']
    yaw=np.linspace(*planner.limits[0],options.get('base_samples',31))
    low,high=options['camera_height_range']
    joints=[];positions=[];rotations=[]
    for posture in itertools.product(*(grid[f'joint{i}'] for i in range(2,7))):
        for angle in yaw:
            q=np.r_[angle,posture,planner.gripper]
            if np.any(q[:6]<planner.limits[:,0]) or np.any(q[:6]>planner.limits[:,1]):continue
            eye,axes=planner.pose(q)
            mujoco.mj_forward(planner.model,planner.data)
            gripper=(planner.data.body('link7').xpos+planner.data.body('link8').xpos)/2
            if (low<=eye[2]<=high and axes[2,2]>planner.min_downward_cosine
                    and np.linalg.norm(gripper[:2])<=options['max_gripper_radius']):
                joints.append(q);positions.append(eye);rotations.append(axes)
    if not joints:raise ValueError('No compact survey postures satisfy the installation limits.')
    planner.pool=np.asarray(joints);planner.positions=np.asarray(positions);planner.rotations=np.asarray(rotations)


def generate(config, directory, max_views=None, *, allow_partial=False):
    import mujoco
    from piper_mujoco_sim.inspection import InspectionPlanner
    from piper_mujoco_sim.survey import generate as generate_survey
    result=copy.deepcopy(config)
    model_config=copy.deepcopy(config)
    resolve_planner_paths(model_config,directory)
    options=model_config['view_planner']
    if max_views is None:
        max_views=options.get('survey_motion',{}).get('max_views',32)
    if not options.get('installation'):
        raise ValueError('Hardware survey generation requires measured installation camera transforms.')
    scene=json.loads(Path(options['scene_config']).read_text())
    regions,rotation=table_regions(scene)
    result.update(regions)
    model=mujoco.MjModel.from_xml_string(installation_xml(model_config))
    scan=result['scan']
    initial=options['installation']['reference_joints']
    compact=options.get('survey_motion',{}).get('strategy')=='compact_rotation'
    seeds=[initial] if compact else list(scan['targets'].values())
    planner=InspectionPlanner(model,seeds,None,gripper=scan.get('gripper_position',.035),
                              fixed_wrist_roll=False,allow_inverted=True,rgb_mask_clearance=True,
                              sample_count=1 if compact else options.get('survey_sample_count',options.get('sample_count',20000)))
    if compact:compact_rotation_pool(planner,options['survey_motion'])
    near=options.get('near_post_survey',{})
    excluded_radius=near.get('radius',.25) if near.get('exclude_from_search',False) else None
    if excluded_radius is not None:
        result['excluded_regions']=[{'type':'circle','center':[0.,0.],'radius':excluded_radius,
                                    'reason':'Competition no-block zone around the arm post'}]
    else:
        result['excluded_regions']=[]
    targets,report=generate_survey(model,scene,{},max_views,planner=planner,
        origin=[*scene['post_position'],0.],initial=initial,transit=initial,max_candidates=1024,
        proposal_only=True,include_overhead=True,map_rotation=rotation,usable_rgb=True,
        camera_intrinsics=options['installation']['cameras'],minimum_table_fraction=.4 if compact else .65,
        broad_radius=options['survey_motion'].get('broad_radius',.25) if compact else None,
        allow_partial=allow_partial,excluded_radius=excluded_radius)
    targets=(order_rotation_sweep(planner,targets,initial,options['survey_motion'].get('max_base_step')) if compact
             else order_survey(planner,targets,initial))
    if not near.get('enabled',False):
        # Count every saved stop, including intermediate cobra angles. The
        # greedy key-view report otherwise understates broad-only coverage.
        _,report=generate_survey(model,scene,{},planner=planner,
            origin=[*scene['post_position'],0.],initial=initial,overhead_pose=initial,
            proposal_only=True,include_overhead=True,map_rotation=rotation,usable_rgb=True,
            camera_intrinsics=options['installation']['cameras'],minimum_table_fraction=.4 if compact else .65,
            prior_views=targets,evaluate_only=True,allow_partial=allow_partial,
            broad_radius=options['survey_motion'].get('broad_radius',.25) if compact else None,
            excluded_radius=excluded_radius)
    labels=[f'sweep_{i+1}' for i in range(len(targets))]
    stages={'broad':list(labels),'near_post':[]}
    if near.get('enabled',False):
        import numpy as np
        broad_report={k:v for k,v in report.items() if k!='geometry'}
        near_planner=InspectionPlanner(model,[initial,*targets],None,
            gripper=scan.get('gripper_position',.035),fixed_wrist_roll=False,allow_inverted=True,
            rgb_mask_clearance=True,sample_count=1 if near.get('fixed_posture',False) else options.get('survey_sample_count',60000))
        if near.get('fixed_posture',False):
            compact_rotation_pool(near_planner,near)
        keep=[]
        for i,q in enumerate(near_planner.pool):
            eye,_=near_planner.pose(q)
            gripper=(near_planner.data.body('link7').xpos+near_planner.data.body('link8').xpos)/2
            if (near['camera_height_range'][0]<=eye[2]<=near['camera_height_range'][1]
                    and np.linalg.norm(gripper[:2])<=near['max_gripper_radius']):keep.append(i)
        if not keep:raise ValueError('No near-post survey poses satisfy the installation limits.')
        near_planner.pool=near_planner.pool[keep]
        near_planner.positions=near_planner.positions[keep]
        near_planner.rotations=near_planner.rotations[keep]
        if near.get('posture_grid') and not near.get('fixed_posture',False):
            existing=(near_planner.pool,near_planner.positions,near_planner.rotations)
            compact_rotation_pool(near_planner,near)
            near_planner.pool=np.concatenate([existing[0],near_planner.pool])
            near_planner.positions=np.concatenate([existing[1],near_planner.positions])
            near_planner.rotations=np.concatenate([existing[2],near_planner.rotations])
        extra,report=generate_survey(model,scene,{},near.get('max_views',6),planner=near_planner,
            origin=[*scene['post_position'],0.],initial=targets[-1],max_candidates=1024,
            proposal_only=True,include_overhead=True,map_rotation=rotation,usable_rgb=True,
            camera_intrinsics=options['installation']['cameras'],minimum_table_fraction=.65,
            prior_views=targets,overhead_pose=initial,near_radius=near.get('radius',.25),
            broad_radius=near.get('radius',.25),
            allow_partial=allow_partial)
        extra=order_survey(near_planner,extra,targets[-1])
        stages['near_post']=[f'near_post_{i+1}' for i in range(len(extra))]
        labels+=stages['near_post'];targets+=extra
        report['broad_sweep']=broad_report
        report['minimum_table_image_fraction']=min(.4 if compact else .65,.65)
    result['viewpoints']=labels
    result['survey_stages']=stages
    scan['targets']=dict(zip(labels,targets))
    scan.update(move_seconds=.8,settle_seconds=.25,max_joint_speed=.6,max_joint_acceleration=1.2,
        motion_profiles={
            'survey':{'move_seconds':.8,'settle_seconds':.25,'max_joint_speed':.25,'max_joint_acceleration':.5},
            'near_post':{'move_seconds':3.,'settle_seconds':1.,'max_joint_speed':.15,
                         'max_joint_acceleration':.15,'max_joint_jerk':.3},
            'candidate_inspection':{'move_seconds':3.,'settle_seconds':1.,
                                    'max_joint_speed':.15,'max_joint_acceleration':.15,
                                    'max_joint_jerk':.3}})
    result['stationary_seconds']=.25
    result['view_planner']['survey_sample_count']=options.get('survey_sample_count',options.get('sample_count',20000))
    result['view_planner']['sample_count']=6000
    result['survey_geometry']=installation_geometry(model_config,model)
    result['survey_coverage']={k:v for k,v in report.items() if k!='geometry'}
    # Survey finds locations to visit. Only focused close-up wrist captures
    # measure the geometry used to confirm a block and its pose.
    result['detector']['proposal_only_viewpoints']=labels
    result['detector']['overhead_proposals_only']=True
    result['candidate_focus']={'enabled':True,'max_views_per_candidate':4}
    result['inspection_rounds']=[[f'refine_{i+1}' for i in range(32)]]
    result['search_criteria']={'min_explored_fraction':.9,'min_resolved_views':2,
        'min_wrist_views':2,'max_position_uncertainty':.01,
        'max_yaw_uncertainty':.2617993877991494,'min_coverage_gain':.005,
        'max_no_progress_rounds':2}
    use_tabletop_frame(result,scene,rotation)
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config',type=Path,required=True)
    parser.add_argument('--max-views',type=int,default=None,
                        help='Override survey_motion.max_views (fallback: 32).')
    parser.add_argument('--write',action='store_true')
    parser.add_argument('--allow-partial',action='store_true',
        help='Save the best valid survey with explicit uncovered points when 95%% coverage is infeasible.')
    args=parser.parse_args()
    config=generate(json.loads(args.config.read_text()),args.config.parent,args.max_views,
                    allow_partial=args.allow_partial)
    if args.write:args.config.write_text(json.dumps(config,indent=2)+'\n')
    print(json.dumps({'views':len(config['viewpoints']),
                      'coverage':config['survey_coverage'],'written':args.write},indent=2))


if __name__=='__main__':main()
