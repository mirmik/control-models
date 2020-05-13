#!/usr/bin/env python3
from zencad import *

a = zencad.assemble.unit([cylinder(r=3, h=6)])
b = zencad.assemble.rotator(parent=a, axis=(0,0,1))
c = zencad.assemble.unit([cylinder(r=3, h=60, center=True)
#	+sphere(10).up(-40)
], location=up(3) * rotateX(deg(90)), parent=b.output)

a.relocate(rotateX(deg(90)), deep=True)

simulation = zencad.bullet.simulation(scale_factor=1, gravity=(0,0,0), substeps=20)
simulation.add_assemble(a, fixed_base=True)

#servo = zencad.bullet.speed_controller(kunit=b, kp=0.000000001, simulation=simulation)
#servo.set_target(10)

servo = zencad.bullet.servo_controller3(simulation=simulation, kunit=b)
servo.set_regs(
	pidspd=zencad.bullet.pid.by_params(T=0.5, ksi=8, M=200), 
	pidpos=zencad.bullet.pid.by_params(T=3, ksi=32), 
	filt=1)


def animate(state):
	servo.set_position_target(math.sin(state.time)*math.pi)
	#servo.set_position_target(math.pi)
	#servo.set_speed_target(math.pi)
	#servo.serve_spd_only(state.delta)
	#print(servo.speed())
	servo.serve(state.delta)
	simulation.step()

show(animate=animate)