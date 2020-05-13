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

simulation = zencad.libs.bullet.pybullet_simulation(
	gravity=(0,0,-10), gui=False, plane=True, scale_factor=1000)

S = 10

#model_colision = simulation.volumed_collision(MODEL.scale(0.5*S))
model_colision = MODEL.scale(0.5)

simulation.add_body(box(5*S).rotateX(deg(60)).left(30*S), translate(0,0,10*S))
#simulation.add_body(sphere(5), translate(0,0,10))
simulation.add_body(MODEL.scale(0.5*S), translate(0,0,0), collision=model_colision)
simulation.add_body(MODEL.scale(0.5*S), translate(0,0,40*S) * rotateZ(deg(60)), collision=model_colision)
simulation.add_body(MODEL.scale(0.5*S), translate(0,0,60*S) * rotateZ(deg(60)), collision=model_colision)
simulation.add_body(box(5*S).rotateX(deg(60)), translate(0,0,80*S))
#obj = simulation.add_body(cylinder(r=5, h=100), translate(40,40,50) * rotateX(deg(60)))

V = 160

aaa = simulation.add_body(sphere(5*S), translate(0,-200*S,10*S))
aaa.set_velocity((0,V*S,0))

aaa = simulation.add_body(sphere(5*S), translate(-220*S,10*S,10*S))
aaa.set_velocity((V*S,0,0))


aaa = simulation.add_body(sphere(5*S), translate(-300*S,-300*S,10*S))
aaa.set_velocity((V*S,V*S,0))

def animate(wdg):
	simulation.step()

show(animate=animate)


#while True:
#	simulation.step()
#	time.sleep(1/240)
