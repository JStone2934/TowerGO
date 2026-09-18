import math
from types import SimpleNamespace as S
import pytest
from astribot_nav_bridge.contracts import checked_command,inspect_state

def test_limits_and_nonholonomic_contract():
    assert checked_command([1,0,0,0,0,-1])==(.2,-.3)
    assert checked_command([-1,0,0,0,0,0])==(0.,0.)
    for i in (1,2,3,4):
        values=[0.]*6;values[i]=.1
        with pytest.raises(ValueError):checked_command(values)
@pytest.mark.parametrize('bad',[float('nan'),float('inf'),-float('inf')])
def test_nonfinite_rejected(bad):
    with pytest.raises(ValueError):checked_command([bad,0,0,0,0,0])
def test_state_does_not_invent_coordinate_semantics():
    m=S(header=S(stamp=S(sec=123,nanosec=456),frame_id=''),name=[],position=[1,2,3],velocity=[4,5,6])
    out=inspect_state(m)
    assert out['stamp']=={'sec':123,'nanosec':456}
    assert out['position']==[1,2,3] and not out['usable_as_odometry']
    m.velocity=[1,2]
    with pytest.raises(ValueError):inspect_state(m)
