"""Synthetic geometry only. No dimensions or calibration from the physical robot."""
import math
import numpy as np

SCENARIOS = ('room', 'corridor', 'obstacles')

def geometry(scenario):
    if scenario not in SCENARIOS:
        raise ValueError(f'Unknown scenario: {scenario}')
    boundary = [(-2,-2),(6,-2),(6,6),(-2,6)]
    if scenario == 'corridor':
        boundary = [(-2,-2),(6,-2),(6,1),(1,1),(1,6),(-2,6)]
    polygons = [boundary]
    if scenario == 'obstacles':
        polygons += [[(1.5,1.5),(2.5,1.5),(2.5,2.5),(1.5,2.5)]]
    return np.array([(p[i],p[(i+1)%len(p)]) for p in polygons for i in range(len(p))],dtype=float)

def route(scenario):
    return [(4.,0.),(0.,0.),(0.,4.),(0.,0.)] if scenario=='corridor' else [(4.,0.),(4.,4.),(0.,4.),(0.,0.)]

def goals(scenario):
    return [(3.,0.,0.),(0.,0.,math.pi/2),(0.,3.,math.pi/2)] if scenario=='corridor' else [(3.,0.,0.),(3.,3.,math.pi/2),(0.,3.,math.pi)]

def wrap(angle):
    return math.atan2(math.sin(angle),math.cos(angle))

def advance(pose, v, w, dt):
    x,y,a=pose
    if abs(w)<1e-9:
        return np.array([x+v*dt*math.cos(a),y+v*dt*math.sin(a),wrap(a+w*dt)])
    return np.array([x+v/w*(math.sin(a+w*dt)-math.sin(a)),y-v/w*(math.cos(a+w*dt)-math.cos(a)),wrap(a+w*dt)])

def raycast(origin, angles, segments, max_range=12.):
    d=np.stack((np.cos(angles),np.sin(angles)),axis=1)
    a=segments[:,0]-np.asarray(origin)
    s=segments[:,1]-segments[:,0]
    cross=lambda u,v:u[...,0]*v[...,1]-u[...,1]*v[...,0]
    den=cross(d[:,None,:],s[None,:,:])
    valid=np.abs(den)>1e-10
    den=np.where(valid,den,1.)
    t=cross(a,s)[None,:]/den
    u=cross(a[None,:,:],d[:,None,:])/den
    t=np.where(valid&(t>=0)&(u>=0)&(u<=1),t,np.inf)
    out=t.min(axis=1)
    return np.where(out<=max_range,out,np.inf)

def wall_distances(points,segments):
    p=np.asarray(points)[:,None,:]; a=segments[None,:,0,:]; d=segments[None,:,1,:]-a
    t=np.clip(np.sum((p-a)*d,axis=2)/np.sum(d*d,axis=2),0,1)
    return np.linalg.norm(p-(a+t[:,:,None]*d),axis=2).min(axis=1)

def collides(pose,segments):
    # Exact segment/rectangle intersection in robot frame, including corners.
    x,y,a=pose; c,s=math.cos(a),math.sin(a)
    seg=(segments-np.array([x,y]))@np.array([[c,-s],[s,c]])
    for p,q in seg:
        delta=q-p; lo,hi=0.,1.
        for axis,half in ((0,.4),(1,.35)):
            if abs(delta[axis])<1e-12:
                if abs(p[axis])>half: lo,hi=1.,0.; break
            else:
                t1=(-half-p[axis])/delta[axis]; t2=(half-p[axis])/delta[axis]
                lo=max(lo,min(t1,t2)); hi=min(hi,max(t1,t2))
        if lo<=hi: return True
    return False
