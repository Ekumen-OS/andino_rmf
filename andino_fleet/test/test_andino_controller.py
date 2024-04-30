import pytest
import rclpy
from andino_fleet.andino_controller_server import AndinoControllerServer
from geometry_msgs.msg import Twist
import random
import math
import time

rclpy.init()
@pytest.fixture
def setup():
    return AndinoControllerServer('andino_server')

@pytest.mark.parametrize("i", range(10))
def test_normalize_angle(setup, i):
    angle = (random.random()-0.5)*2*math.pi  #angles between [-pi,pi)
    new_angle = angle + (random.randint(0,10)-5)*2*math.pi  #random angles between [-10pi, 10pi]
    norm_angle = setup._normalize_angle(new_angle)
    assert norm_angle == pytest.approx(angle,abs=1e-4)

    

