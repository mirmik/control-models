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

simulation = zencad.libs.bullet.pybullet_simulation(gravity=(0,0,-10), gui=False, plane=True)

#model_colision = simulation.volumed_collision(MODEL.scale(0.5))
#simulation.add_body(MODEL.scale(0.5), translate(0,0,0), collision=model_colision)
simulation.add_body(MODEL.scale(0.5), translate(0,0,0))

def animate(wdg):
	simulation.step()

show(animate=animate)
