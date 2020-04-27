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

simulation = zencad.libs.bullet.pybullet_simulation(gravity=(0,0,-1000), plane=True)

model_colision = zencad.libs.bullet.volumed_collision(MODEL.scale(0.5))

simulation.add_body(box(5).rotateX(deg(60)).left(30), translate(0,0,10))
simulation.add_body(sphere(5), translate(0,0,10))
simulation.add_body(MODEL.scale(0.5), translate(0,0,0), collision=model_colision)
simulation.add_body(MODEL.scale(0.5), translate(0,0,40) * rotateZ(deg(60)), collision=model_colision)
simulation.add_body(MODEL.scale(0.5), translate(0,0,60) * rotateZ(deg(60)), collision=model_colision)
simulation.add_body(box(5).rotateX(deg(60)), translate(0,0,80), mass=2000)
obj = simulation.add_body(cylinder(r=5, h=100), translate(40,40,50) * rotateX(deg(60)))

aaa = simulation.add_body(sphere(5), translate(0,-300,10), mass=2000)
aaa.set_velocity((0,400,0))

aaa = simulation.add_body(sphere(5), translate(-320,10,10), mass=2000)
aaa.set_velocity((400,0,0))


aaa = simulation.add_body(sphere(5), translate(-400,-400,10), mass=2000)
aaa.set_velocity((400,400,0))

a=True
starttime=time.time()
def animate(wdg):
	global a
	curtime = time.time()
	simulation.step()

show(animate=animate)
