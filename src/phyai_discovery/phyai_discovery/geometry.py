"""Measured convex tabletop regions in the configured map frame."""
import numpy as np


def outside_excluded_regions(points, regions=()):
    """Search exclusions use candidate/cell centres in the declared map frame."""
    points=np.asarray(points)
    keep=np.ones(points.shape[:-1],dtype=bool)
    for region in regions or ():
        center=np.asarray(region.get('center'),dtype=float)
        radius=region.get('radius')
        if (region.get('type')!='circle' or center.shape!=(2,) or not np.isfinite(center).all()
                or isinstance(radius,bool) or not isinstance(radius,(int,float))
                or not np.isfinite(radius) or radius<=0):
            raise ValueError('Excluded search regions require circle type, finite XY center and positive radius.')
        keep &= np.linalg.norm(points[...,:2]-center,axis=-1)>=radius
    return keep


def inside_polygon(points,polygon):
    points=np.asarray(points)
    if polygon is None:return np.ones(points.shape[:-1],dtype=bool)
    vertices=np.asarray(polygon,dtype=float)
    edge=np.roll(vertices,-1,axis=0)-vertices
    delta=points[...,None,:2]-vertices
    cross=edge[:,0]*delta[...,1]-edge[:,1]*delta[...,0]
    return np.all(cross>=-1e-9,axis=-1)|np.all(cross<=1e-9,axis=-1)


def footprint_inside(position,size,quaternion,*,scope=None,polygon=None):
    """Require the entire estimated cuboid footprint inside the measured table."""
    if scope is None and polygon is None:return True
    position=np.asarray(position,dtype=float);size=np.asarray(size,dtype=float)
    q=np.asarray(quaternion,dtype=float)
    if (position.shape!=(3,) or size.shape!=(3,) or q.shape!=(4,)
            or not np.isfinite(np.r_[position,size,q]).all() or (size<=0).any()
            or np.linalg.norm(q)<1e-9):return False
    w,x,y,z=q/np.linalg.norm(q)
    rotation=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                       [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                       [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
    corners=np.array([[x,y,z] for x in (-1,1) for y in (-1,1) for z in (-1,1)])*size/2
    xy=(corners@rotation.T+position)[:,:2]
    if scope is not None:
        xmin,xmax,ymin,ymax=scope
        if not ((xy>=[xmin,ymin])&(xy<=[xmax,ymax])).all():return False
    return bool(inside_polygon(xy,polygon).all())
