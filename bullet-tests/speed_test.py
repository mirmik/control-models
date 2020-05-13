#!/usr/bin/env python3

import pybullet as p
import time
import pybullet_data

import zencad
import zencad.libs.bullet
from zencad import *
from zencad.libs.obj_writer import write_as_obj_wavefront

import tempfile

from micar import MODEL

simulation = zencad.libs.bullet.pybullet_simulation(gravity=(0,0,0), plane=False)

aaa = simulation.add_body(box(5, center=False))
aaa.set_velocity((10,0,0))

for i in range(100):
	disp(point3(10*i,0,0))

print(1/240)
def animate(state):
	#print(state.delta)
	simulation.step()

show(animate=animate, animate_step=1/240)
