#!/usr/bin/env python3

import zencad.libs.bullet

import zencad
from zencad import *

import time


simulation = zencad.libs.bullet.pybullet_simulation(gravity=(0,0,-1000), plane=True, gui=False)
#simulation.set_plane_friction(1)

ma=box(10)
mb=box(10)

a = simulation.add_multibody(
	base_model=ma,
	base_location=translate(0,0,10),
	
	#link_models=[ma],
	#link_parents=[0],
	#link_locations=[
	#	up(2)
	#],
	#link_joint_types=[zencad.libs.bullet.JOINT_FIXED],
	#link_axes=[(0,0,1)]
)


starttime=time.time()
def animate(wdg):
	global a
	curtime = time.time()
	simulation.step()

show(animate=animate)

