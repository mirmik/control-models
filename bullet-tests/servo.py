#!/usr/bin/env python3
from zencad import *
import zencad.bullet

a = zencad.assemble.unit([cylinder(r=3, h=6)])
b = zencad.assemble.rotator(parent=a, axis=(0,0,1))
c = zencad.assemble.unit([cylinder(r=3, h=40, center=True)], location=up(3) * rotateX(deg(90)), parent=b.output)

a.relocate(rotateX(deg(90)), deep=True)

simulation = zencad.bullet.simulation(scale_factor=1000, gravity=(0,0,0))
simulation.add_assemble(a, fixed_base=True)

servo = zencad.bullet.servo_controller3(kunit=b, simulation=simulation)
servo.set_regs(
	zencad.bullet.pid(kp=0.00000002, ki=0.00001),
	zencad.bullet.pid(kp=20, ki=30))
servo.set_speed_target(deg(45))

def animate(state):
	#t = ((math.sin(state.time / 2) + 1)/2) *20 + 2
	#print(t)
	#servo.set_target(t)
	print(servo.speed())
	servo.serve_spd_only(state.delta)
	simulation.step()

show(animate=animate)