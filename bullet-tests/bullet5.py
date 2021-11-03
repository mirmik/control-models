#!/usr/bin/env python3

import zencad.libs.bullet

import zencad
from zencad import *

import time
import pybullet


simulation = zencad.libs.bullet.simulation(
	gravity=(0,0,-10), scale_factor=10, plane=True, gui=False)
#simulation.set_plane_friction(1)

Z = 100

ma=box(Z, center=True)
mb=nullshape()
mbb = sphere(Z/8)
k1 = cylinder(r=Z/8, h=Z/4*3).rotateX(deg(20))
k2 = cylinder(r=Z/8, h=Z/4*3).rotateX(-deg(20))#.forw(0)

a = simulation.add_multibody(
	base_model=ma,
	base_location=translate(0,0,Z),
	#base_mass=10000,
	
	link_models=[mbb,mb,k1,k2],
	#link_masses=[0,0,100,100],
	link_parents=[0,0,1,2],
	link_locations=[
		move(Z/2,Z/2,Z/2),
		move(Z/2,-Z/2,Z/2), 
		move(0,0,0), 
		move(0,0,0), 
#		move(0,0,5), 
	],
	link_joint_types=[
	zencad.libs.bullet.JOINT_FIXED, zencad.libs.bullet.JOINT_FIXED, 
	zencad.libs.bullet.JOINT_REVOLUTE, zencad.libs.bullet.JOINT_REVOLUTE],
	link_axes=[(0,0,0), (0,0,0), (1,0,0), (1,0,0)]
)


print(pybullet.getJointInfo(a.boxId, 3))
print(pybullet.getJointState(a.boxId, 3))

def animate(state):
	#pybullet.setJointMotorControl2(bodyUniqueId=a.boxId, 
	#		jointIndex=3,
	#		controlMode=pybullet.TORQUE_CONTROL,
	#		force =480 + 200)

	pybullet.setJointMotorControl2(a.boxId, 3, pybullet.VELOCITY_CONTROL, 
		targetVelocity=0, force=0)

	pybullet.setJointMotorControl2(a.boxId, 2, pybullet.VELOCITY_CONTROL, 
		targetVelocity=0, force=0)

	pybullet.setJointMotorControl2(a.boxId, 1, pybullet.VELOCITY_CONTROL, 
		targetVelocity=0, force=0)

	pybullet.setJointMotorControl2(a.boxId, 0
		, pybullet.VELOCITY_CONTROL, 
		targetVelocity=0, force=0)



	#maxForce = 500000000000
	#mode = pybullet.VELOCITY_CONTROL
	#pybullet.setJointMotorControl2(
	#	bodyUniqueId=a.boxId, 
	#	jointIndex=3,
	#	targetVelocity = 20,
	#    controlMode=mode, force=maxForce)
	#print(pybullet.getJointState(a.boxId, 3))

	simulation.step()

show(animate=animate)







# enable collision between lower legs

#    for j in range(p.getNumJoints(quadruped)):
#        print(p.getJointInfo(quadruped, j))
#
#    # 2,5,8 and 11 are the lower legs
#    lower_legs = [2, 5, 8, 11]
#    for l0 in lower_legs:
#        for l1 in lower_legs:
#            if (l1 > l0):
#                enableCollision = 1
#                print("collision for pair", l0, l1,
#                      p.getJointInfo(quadruped, l0) [12],
#                      p.getJointInfo(quadruped, l1) [12], "enabled=", enableCollision)
#                p.setCollisionFilterPair(quadruped, quadruped, 2, 5, enableCollision)