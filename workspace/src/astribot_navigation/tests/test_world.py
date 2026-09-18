import math
import numpy as np
import pytest
from astribot_nav_sim.world import advance,raycast,geometry,collides,wall_distances,SCENARIOS,route

def test_kinematic_units_and_rotation():
    assert np.allclose(advance([0,0,math.pi/2],.2,0,5),[0,1,math.pi/2])
    assert np.allclose(advance([0,0,0],0,.3,2),[0,0,.6])
    assert np.allclose(advance([0,0,0],1,1,math.pi/2),[1,1,math.pi/2])
def test_ray_intersection_and_wall_metric():
    walls=geometry('room')
    assert np.allclose(raycast([0,0],np.array([0,math.pi/2,math.pi]),walls),[6,6,2])
    assert np.allclose(wall_distances([[0,-2],[6,1]],walls),0)
def test_rectangle_collision_including_rotated_corners():
    walls=geometry('room')
    assert not collides([0,0,0],walls)
    assert collides([5.7,0,0],walls)
    assert collides([5.55,0,math.pi/4],walls)
@pytest.mark.parametrize('scenario',SCENARIOS)
def test_scripted_waypoints_are_clear(scenario):
    assert all(not collides([x,y,0],geometry(scenario)) for x,y in route(scenario))
def test_seed_repeatability():
    a=np.random.default_rng(42).normal(0,.01,100)
    b=np.random.default_rng(42).normal(0,.01,100)
    assert np.array_equal(a,b)
