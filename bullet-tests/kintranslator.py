#!/usr/bin/env python3

import zencad.libs.bullet

import zencad
from zencad import *

import time
import pybullet


simulation = zencad.libs.bullet.pybullet_simulation(
	gravity=(0,0,-100), plane=False, gui=False, scale_factor=1, substeps=10)

simulation.add_body(box(1000,1000,1,center=True).down(0.5), mass=0)

base = assemble.unit([box(20,70,20,center=True).fillet(3)])
#sunit = assemble.unit([], location=move(10,20,10), parent=base)
#aunit = assemble.unit([sphere(2)], location=move(10,20,10), parent=sunit)
#bunit = assemble.unit([sphere(10)], location=move(10,20,20), parent=sunit)

base.add(sphere(r=10).back(40))

Z = -1
Z2 =-1.3
R = 12
K=12

r0 = zencad.assemble.rotator(parent=base, location=forw(40), axis=(0,0,1))
r1 = zencad.assemble.rotator(parent=r0.output, location=nulltrans(), axis=(1,0,0))

b2 = assemble.unit([box(20,70,20,center=True).fillet(3)], parent=r1.output, location=forw(40))

r01 = zencad.assemble.rotator(parent=b2, location=forw(40), axis=(0,0,1))
r11 = zencad.assemble.rotator(parent=r01.output, location=nulltrans(), axis=(1,0,0))
b3 = assemble.unit([box(20,70,20,center=True).fillet(3)], parent=r11.output, location=forw(40))

rot0 = assemble.rotator(location=move(2*K,2*K,Z2), axis=(1,0,0), parent=base)
rot0.output.add(cylinder(r=R, h=5, center=True).rotX(deg(45)))

rot1 = assemble.rotator(location=move(-2*K,-2*K,Z2), axis=(1,0,0), parent=base)
rot1.output.add(cylinder(r=R, h=5, center=True).rotX(deg(45)))

rot2 = assemble.rotator(location=move(-2*K,2*K,Z2), axis=(1,0,0), parent=base)
rot2.output.add(cylinder(r=R, h=5, center=True).rotX(deg(45)))

rot3 = assemble.rotator(location=move(2*K,-2*K,Z2), axis=(1,0,0), parent=base)
rot3.output.add(cylinder(r=R, h=5, center=True).rotX(deg(45)))



rot01 = assemble.rotator(location=move(2*K,2*K,Z2), axis=(1,0,0), parent=b2)
rot01.output.add(cylinder(r=R, h=5, center=True).rotX(deg(45)))

rot11 = assemble.rotator(location=move(-2*K,-2*K,Z2), axis=(1,0,0), parent=b2)
rot11.output.add(cylinder(r=R, h=5, center=True).rotX(deg(45)))

rot21 = assemble.rotator(location=move(-2*K,2*K,Z2), axis=(1,0,0), parent=b2)
rot21.output.add(cylinder(r=R, h=5, center=True).rotX(deg(45)))

rot31 = assemble.rotator(location=move(2*K,-2*K,Z2), axis=(1,0,0), parent=b2)
rot31.output.add(cylinder(r=R, h=5, center=True).rotX(deg(45)))



rot02 = assemble.rotator(location=move(2*K,2*K,Z2), axis=(1,0,0), parent=b3)
rot02.output.add(cylinder(r=R, h=5, center=True).rotX(deg(45)))

rot12 = assemble.rotator(location=move(-2*K,-2*K,Z2), axis=(1,0,0), parent=b3)
rot12.output.add(cylinder(r=R, h=5, center=True).rotX(deg(45)))

rot22 = assemble.rotator(location=move(-2*K,2*K,Z2), axis=(1,0,0), parent=b3)
rot22.output.add(cylinder(r=R, h=5, center=True).rotX(deg(45)))

rot32 = assemble.rotator(location=move(2*K,-2*K,Z2), axis=(1,0,0), parent=b3)
rot32.output.add(cylinder(r=R, h=5, center=True).rotX(deg(45)))

base.relocate(up(30))

b = simulation.bind_assemble(base)




print(1/240)
def animate(state):
#	print(state.delta)
	F = 20000000
	F2 = 18000000
	simulation.set_force(b, rot0.simulation_hint, F)
	simulation.set_force(b, rot1.simulation_hint, F)
	simulation.set_force(b, rot2.simulation_hint, F)
	simulation.set_force(b, rot3.simulation_hint, F)
	simulation.set_force(b, rot01.simulation_hint, F2)
	simulation.set_force(b, rot11.simulation_hint, F2)
	simulation.set_force(b, rot21.simulation_hint, F2)
	simulation.set_force(b, rot31.simulation_hint, F2)
	simulation.set_force(b, rot02.simulation_hint, F2)
	simulation.set_force(b, rot12.simulation_hint, F2)
	simulation.set_force(b, rot22.simulation_hint, F2)
	simulation.set_force(b, rot32.simulation_hint, F2)
	simulation.step()

show(animate=animate)
