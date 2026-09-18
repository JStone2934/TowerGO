"""Pure validation, intentionally independent of Astribot SDK and ROS imports."""
import math

def checked_command(values, max_v=.2, max_w=.3):
    if len(values)!=6 or not all(math.isfinite(float(x)) for x in values):
        raise ValueError('Twist must contain six finite numbers')
    if any(abs(values[i])>1e-9 for i in (1,2,3,4)):
        raise ValueError('Only forward motion and yaw are enabled')
    return max(0.,min(max_v,values[0])),max(-max_w,min(max_w,values[5]))

def inspect_state(msg):
    p=list(msg.position); v=list(msg.velocity)
    if len(p)!=3 or len(v)!=3 or not all(math.isfinite(x) for x in p+v):
        raise ValueError('Expected three finite position and velocity values')
    return {'stamp':{'sec':msg.header.stamp.sec,'nanosec':msg.header.stamp.nanosec},
            'frame_id':msg.header.frame_id,'name':list(msg.name),'position':p,'velocity':v,
            'semantics':'unconfirmed','usable_as_odometry':False}
