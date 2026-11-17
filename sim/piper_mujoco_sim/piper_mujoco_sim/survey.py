"""Shared survey generation from measured table/robot geometry, without block truth."""
import numpy as np
from .inspection import InspectionPlanner, scan_geometry


def camera_frame(model,eye,axes,intrinsics=None,name='wrist'):
    from types import SimpleNamespace
    transform=np.eye(4);transform[:3,:3]=axes@np.diag([1.,-1.,-1.]);transform[:3,3]=eye
    focal=480/(2*np.tan(np.deg2rad(float(model.camera(name).fovy[0]))/2))
    k=intrinsics or dict(width=640,height=480,fx=focal,fy=focal,cx=319.5,cy=239.5)
    return SimpleNamespace(camera_to_map=transform,intrinsics=SimpleNamespace(**{
        key:k[key] for key in ('width','height','fx','fy','cx','cy')}))


def usable_samples(planner,q,points,corners,intrinsics=None,name='wrist'):
    """Full proposal footprints must fit and clear the RGB robot exclusion."""
    import mujoco
    from .robot_mask import robot_mask
    planner.pose(q);mujoco.mj_forward(planner.model,planner.data)
    camera=planner.data.camera(name)
    frame=camera_frame(planner.model,camera.xpos,camera.xmat.reshape(3,3),intrinsics,name)
    local=(points[:,None,:]+corners-frame.camera_to_map[:3,3])@frame.camera_to_map[:3,:3]
    k=frame.intrinsics
    pixels=local[:,:,:2]/np.maximum(local[:,:,2,None],1e-8)*[k.fx,k.fy]+[k.cx,k.cy]
    low=np.floor(pixels.min(axis=1)).astype(int);high=np.ceil(pixels.max(axis=1)).astype(int)
    valid=(local[:,:,2]>.08).all(axis=1)&(low>=4).all(axis=1)&(high[:,0]<k.width-4)&(high[:,1]<k.height-4)
    mask=robot_mask(planner.model,planner.data,frame)
    integral=np.pad(mask.astype(np.int64),((1,0),(1,0))).cumsum(0).cumsum(1)
    ids=np.flatnonzero(valid);x0,y0=low[ids].T;x1,y1=(high[ids]+1).T
    blocked=integral[y1,x1]-integral[y0,x1]-integral[y1,x0]+integral[y0,x0]
    valid[ids]&=blocked==0
    return valid


def table_image_fractions(positions,rotations,scene,origin,map_rotation,intrinsics):
    """Fraction of sampled image rays that hit the physical tabletop."""
    k=intrinsics
    u,v=np.meshgrid(np.linspace(0,k['width']-1,13),np.linspace(0,k['height']-1,11))
    rays=np.column_stack(((u.ravel()-k['cx'])/k['fx'],-(v.ravel()-k['cy'])/k['fy'],-np.ones(u.size)))
    direction=np.einsum('pi,nji->npj',rays,rotations)
    distance=(.015-positions[:,None,2])/np.minimum(direction[:,:,2],-1e-9)
    hits=positions[:,None,:]+distance[:,:,None]*direction
    # Convert hits back to scene coordinates to test all four physical plates.
    hits=hits@map_rotation+np.asarray(origin)
    half=np.asarray(scene['table_size'])+scene['table_gap']/2
    on=(direction[:,:,2]<-1e-6)&(distance>0)&(np.abs(hits[:,:,:2])<=half).all(axis=2)
    return on.mean(axis=1)


def generate(model, scene, config, max_views=24, *, planner=None, origin=(0.,0.,0.), initial=None,
             transit=None, max_candidates=160, proposal_only=False, include_overhead=False,
             map_rotation=None,usable_rgb=False,camera_intrinsics=None,minimum_table_fraction=0.,
             allow_partial=False,broad_radius=None,near_radius=None,prior_views=(),overhead_pose=None,
             excluded_radius=None,evaluate_only=False):
    planner = planner or InspectionPlanner(model, list(config['sim_scan']['targets'].values()),
                                None, gripper=config['sim_scan'].get('gripper_position', .035))
    # Grid of possible top faces on all plates. Actual block positions/count and
    # colours never enter the target grid, scoring, visibility or path checks.
    width, depth = scene['table_size']
    gap = scene['table_gap']
    points = []
    plates = []
    for row in range(2):
        for column in range(2):
            low = np.array([(column - .5)*(width + gap) - width/2,
                            (row - .5)*(depth + gap) - depth/2])
            for x in np.arange(low[0] + .06, low[0] + width - .04, .08):
                for y in np.arange(low[1] + .06, low[1] + depth - .04, .08):
                    if np.linalg.norm(np.array([x, y]) - scene['post_position']) > .10:
                        points.append([x, y, .015])
                        plates.append(row*2 + column)
    # Dense ring beside the post: a coarse plate average can hide this blind spot.
    for dx in np.arange(-.24, .241, .04):
        for dy in np.arange(-.24, .241, .04):
            if .10 <= np.hypot(dx, dy) <= .245:
                x, y = np.asarray(scene['post_position']) + [dx, dy]
                points.append([x, y, .015])
                plates.append(int(y >= 0)*2 + int(x >= 0))
    map_rotation=np.eye(3) if map_rotation is None else np.asarray(map_rotation)
    points, plates = (np.asarray(points)-np.asarray(origin))@map_rotation.T, np.asarray(plates)
    if excluded_radius is not None:
        if not np.isfinite(excluded_radius) or excluded_radius<=0:
            raise ValueError('Excluded post radius must be finite and positive.')
        keep=np.linalg.norm(points[:,:2],axis=1)>=excluded_radius
        points,plates=points[keep],plates[keep]
        if not len(points):raise ValueError('Excluded post radius removes the entire search area.')
    broad=np.ones(len(points),bool) if broad_radius is None else np.linalg.norm(points[:,:2],axis=1)>=broad_radius
    weights=np.where(broad,1.,.1)
    if near_radius is not None:weights=np.where(np.linalg.norm(points[:,:2],axis=1)<near_radius,1.,.05)
    corners = np.array([[x, y, 0] for x in (-.0375, .0375) for y in (-.0125, .0125)] +
                       [[x, y, 0] for x in (-.0125, .0125) for y in (-.0375, .0375)])
    if proposal_only and not usable_rgb:
        corners=np.zeros((1,3))  # Locate centres first; close views measure full silhouettes.
    corners=corners@map_rotation.T
    tangent = np.tan(np.deg2rad(float(model.camera('wrist').fovy[0]))/2)
    # Bound temporary memory when calibrated hardware needs a denser pose pool.
    framed=np.empty((len(planner.pool),len(points)),dtype=bool)
    for start in range(0,len(planner.pool),256):
        rotations=planner.rotations[start:start+256]
        positions=planner.positions[start:start+256]
        local=(np.einsum('pki,nij->npkj',points[:,None,:]+corners,rotations)-
               np.einsum('ni,nij->nj',positions,rotations)[:,None,None,:])
        distance=-local[:,:,:,2]
        framed[start:start+256]=np.all((distance>.08)&(abs(local[:,:,:,0])<distance*tangent*4/3)&
                                      (abs(local[:,:,:,1])<distance*tangent),axis=2)
    fractions=np.ones(len(planner.pool))
    if usable_rgb:
        fractions=table_image_fractions(planner.positions,planner.rotations,scene,origin,map_rotation,camera_intrinsics['wrist'])
        framed[fractions<minimum_table_fraction]=False
    visible_cache={}
    previous = list(initial) if initial is not None else [*scene['home_joints'], planner.gripper]
    covered = np.zeros(len(points), dtype=bool)
    if include_overhead:
        import mujoco
        overhead_joints=previous if overhead_pose is None else overhead_pose
        planner.pose(overhead_joints);mujoco.mj_forward(planner.model,planner.data)
        camera=planner.data.camera('overhead')
        eye=camera.xpos.copy();axes=camera.xmat.reshape(3,3)
        local=(points-eye)@axes;distance=-local[:,2]
        tangent_overhead=np.tan(np.deg2rad(float(model.camera('overhead').fovy[0]))/2)
        visible=(distance>.08)&(abs(local[:,0])<distance*tangent_overhead*4/3)&(abs(local[:,1])<distance*tangent_overhead)
        if usable_rgb:
            visible&=usable_samples(planner,overhead_joints,points,corners,camera_intrinsics['overhead'],'overhead')
        for index in np.flatnonzero(visible):
            ray=points[index]-eye;length=np.linalg.norm(ray)
            hit=mujoco.mj_ray(planner.model,planner.data,eye,ray/length,
                planner.ray_groups,1,model.camera('overhead').bodyid[0],np.zeros(1,dtype=np.int32))
            covered[index]=hit<0 or hit>=length-.015
    overhead_count=int(covered.sum())
    for q in prior_views:
        available=usable_samples(planner,q,points,corners,camera_intrinsics['wrist'])
        for index in np.flatnonzero(available & ~covered):
            if planner._visible(q,points[index],.045,points[index]+corners):covered[index]=True
    used, targets = set(), [list(q) for q in prior_views] if evaluate_only else []
    rates = [float(covered[plates == plate].mean()) for plate in range(4)]
    for _ in range(0 if evaluate_only else max_views):
        scores = ((framed & ~covered)@weights)*fractions
        scores[list(used)] = -1
        best = None
        for index in np.argsort(scores)[::-1][:max_candidates]:
            if scores[index] <= 0:
                break
            if best is not None and scores[index]<=weights[best[1]].sum()*fractions[best[0]]:
                break
            q = planner.pool[index].tolist()
            via_transit=False
            if not planner._clear_path(previous, q):
                if (transit is None or not planner._clear_path(previous,transit)
                        or not planner._clear_path(transit,q)):
                    continue
                via_transit=True
            if transit is not None and not planner._clear_path(q,transit):
                continue
            available=framed[index] & ~covered
            if usable_rgb:
                if index not in visible_cache:
                    visible_cache[index]=usable_samples(planner,q,points,corners,camera_intrinsics['wrist'])
                available &= visible_cache[index]
            seen = [p for p in np.flatnonzero(available)
                    if planner._visible(q, points[p], .045, points[p] + corners)]
            if seen and (best is None or weights[seen].sum()*fractions[index] > weights[best[1]].sum()*fractions[best[0]]):
                best = (index, seen, via_transit)
        if best is None:
            break
        index, seen, via_transit = best
        covered[seen] = True
        used.add(index)
        if via_transit:targets.append(list(transit))
        previous = planner.pool[index].tolist()
        targets.append(previous)
        rates = [float(covered[plates == plate].mean()) for plate in range(4)]
        if min(rates) >= .98:
            break
    if not targets or (min(rates) < .95 and not allow_partial):
        raise ValueError(f'Could not find a survey covering at least 95% of each plate after {len(targets)} views '
                         f'from {len(planner.pool)} candidate poses: {rates}; '
                         f'uncovered sample positions: {np.round(points[~covered],4).tolist()}')
    return targets, {'plate_coverage': rates, 'sample_count': len(points),
                     'overhead_samples':overhead_count,'proposal_only':proposal_only,
                     'rgb_mask_checked':usable_rgb,'minimum_table_image_fraction':minimum_table_fraction,
                     'meets_minimum_plate_coverage':min(rates)>=.95,
                     'broad_radius':broad_radius,
                     'near_radius':near_radius,
                     'excluded_radius':excluded_radius,
                     'broad_plate_coverage':[float(covered[(plates==plate)&broad].mean()) for plate in range(4)],
                     'uncovered_points':points[~covered].tolist(),
                     'geometry': scan_geometry(model)}
