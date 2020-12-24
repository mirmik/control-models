#!/usr/bin/env python3

import zencad.libs.bullet

import zencad
from zencad import *

import time


simulation = zencad.libs.bullet.simulation(scale_factor=1, gravity=(0,0,-1000), plane=True)
#simulation.set_plane_friction(1)

model=cylinder(r=30,h=5).rotX(deg(80)).up(30)

a = simulation.add_body(model)
a.set_velocity(lin=(-70,0,0), ang=(0,-10,0))


starttime=time.time()
def animate(wdg):
	global a
	curtime = time.time()
	simulation.step()

show(animate=animate)

