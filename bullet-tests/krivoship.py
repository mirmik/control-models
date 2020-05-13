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
	gravity=(0,0,0), gui=False, plane=False, scale_factor=10)

class ddd(zencad.assemble.unit):
	def __init__(self, loc):
		super().__init__()
		self.relocate(loc)
		self.add(box(10,10,5,center=True).down(5))
		r = zencad.assemble.rotator(axis=(0,0,1), parent=self, location=down(5))
		r.output.add(cylinder(r=4,h=10))



#W=7.3
W=7.1
simulation.bind_assemble(ddd(move(70,W,3)),fixed_base=True)
simulation.bind_assemble(ddd(move(70,-W,3)),fixed_base=True)
simulation.bind_assemble(ddd(move(180,W,3)),fixed_base=True)
simulation.bind_assemble(ddd(move(180,-W,3)),fixed_base=True)

a = zencad.assemble.unit([cylinder(r=4,h=10)])
r = zencad.assemble.rotator(axis=(0,0,1), parent=a).up(5)
r.set_coord(deg(30))

b= zencad.assemble.unit([cylinder(r=3, h=20).rotX(deg(90))], parent=r.output)
r2 = zencad.assemble.rotator(axis=(0,0,1), parent=b, location=back(20))
r2.set_coord(deg(80))

c = zencad.assemble.unit([cylinder(r=3, h=40).rotX(deg(90))], parent=r2.output)

r3 = zencad.assemble.rotator(axis=(0,0,1), parent=c, location=back(40))
r3.set_coord(deg(-20))

c = zencad.assemble.unit([cylinder(r=3, h=200).rotX(deg(90))], parent=r3.output)



aaa = simulation.bind_assemble(a, fixed_base=True)


def animate(wdg):
	simulation.set_force(aaa, r.simulation_hint, -10)
	simulation.step()

show(animate=animate)


#while True:
#	simulation.step()
#	time.sleep(1/240)
