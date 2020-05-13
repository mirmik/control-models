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

simulation = zencad.libs.bullet.pybullet_simulation(gravity=(0,0,-10), plane=True)

aaa = simulation.add_body(box(5, center=False), location=up(1000))

for i in range(100):
	disp(point3(0,0,10*i))

def animate(state):
	print(aaa.velocity())
	simulation.step()

show(animate=animate, animate_step=1/240)
