#!/usr/bin/env python3
from zencad import *

a = zencad.assemble.unit([cylinder(r=3, h=6)])
b = zencad.assemble.rotator(parent=a, axis=(0,0,1))
c = zencad.assemble.unit([cylinder(r=3, h=40, center=True)], location=up(3) * rotateX(deg(90)), parent=b.output)

a.relocate(rotateX(deg(90)), deep=True)

simulation = zencad.bullet.simulation(scale_factor=1000, gravity=(0,0,0))
simulation.add_assemble(a, fixed_base=True)

servo = zencad.bullet.speed_controller(kunit=b, kp=0.000000001, simulation=simulation)
servo.set_target(10)

def animate(state):
	t = ((math.sin(state.time / 2) + 1)/2) *20 + 2
	print(t)
	servo.set_target(t)
	servo.serve(state.delta)
	simulation.step()

show(animate=animate)