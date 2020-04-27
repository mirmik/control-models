#!/usr/bin/env python3

import zencad.libs.bullet

import zencad
from zencad import *

import time
from micar import MODEL
#from kran import MODEL


simulation = zencad.libs.bullet.pybullet_simulation(gravity=(0,0,0), plane=False)


r0 = cylinder(r=5, h=10)
r1 = cylinder(r=3, h=10)

#simulation.add_multibody(
#	base_visual = r0,
#	base_collision = r0,
#	base_location=nulltrans(),
#
#	links_visuals=[r1],
#	links_collisions=[r1],
#	links_parents=[0],
#	links_locations=[rotateX(deg(45))]
#)

#simulation.add_body(box(10,20,2), location=rotateX(0)  * up(10))

#simulation.add_body(box(5).rotateX(deg(60)).left(30), translate(0,0,10))
#simulation.add_body(sphere(5), translate(0,0,10))
#simulation.add_body(MODEL.scale(0.5), translate(0,0,0), non_convex_collision=True)
#simulation.add_body(MODEL.scale(0.5), translate(0,0,40) * rotateZ(deg(60)), non_convex_collision=True)
#simulation.add_body(MODEL.scale(0.5), translate(0,0,60) * rotateZ(deg(60)), non_convex_collision=True)
#simulation.add_body(box(5).rotateX(deg(60)), translate(0,0,80), mass=5000)
#obj = simulation.add_body(cylinder(r=5, h=100), translate(40,40,50) * rotateX(deg(60)))

W = 80
A = 10
B = 8

#model = box(2,4,20,center=True) + cylinder(r=1, h=10).rotateX(deg(90))
model = (cylinder(r=10,h=5,center=True) 
	+ box(A,center=True).right(10) 
	+ box(A,center=True).left(10)
	+ box(B,center=True).forw(10) 
	+ box(B,center=True).back(10)
)

STEP = 40

a1 = simulation.add_body(model, location=translate(0,STEP + W,0))
a2 = simulation.add_body(model.rotateY(deg(90)), location=translate(STEP,STEP + W,0))
a3 = simulation.add_body(model.rotateX(deg(90)), location=translate(STEP*2,STEP + W,0))
a4 = simulation.add_body(model.rotateX(deg(90)).rotateZ(deg(90)), location=translate(STEP*3,STEP + W,0))
a5 = simulation.add_body(model.rotateY(deg(90)).rotateZ(deg(90)), location=translate(STEP*4,STEP + W,0))
a6 = simulation.add_body(model.rotateY(deg(90)).rotateZ(deg(90)).rotateX(deg(90)), location=translate(STEP*5,STEP + W,0))
print()
a1 = simulation.add_body(model, translate(0,W,0))
a2 = simulation.add_body(model.rotateY(deg(90)), location=translate(STEP,W,0))
a3 = simulation.add_body(model.rotateX(deg(90)), location=translate(STEP*2,W,0))
a4 = simulation.add_body(model.rotateX(deg(90)).rotateZ(deg(90)), location=translate(STEP*3,W,0))
a5 = simulation.add_body(model.rotateY(deg(90)).rotateZ(deg(90)), location=translate(STEP*4,W,0))
a6 = simulation.add_body(model.rotateY(deg(90)).rotateZ(deg(90)).rotateX(deg(90)), location=translate(STEP*5,W,0))

S=40
SV=1
a1.set_velocity(lin=(0,0,0), ang=(SV,S,SV))
a2.set_velocity(lin=(0,0,0), ang=(SV,S,SV))
a3.set_velocity(lin=(0,0,0), ang=(SV,SV,S))
a4.set_velocity(lin=(0,0,0), ang=(SV,SV,S))
a5.set_velocity(lin=(0,0,0), ang=(S,SV,SV))
a6.set_velocity(lin=(0,0,0), ang=(S,SV,SV))



model = model.rotateZ(deg(90))

STEP = 40

a1 = simulation.add_body(model, location=translate(0,STEP,0))
a2 = simulation.add_body(model.rotateY(deg(90)), location=translate(STEP,STEP,0))
a3 = simulation.add_body(model.rotateX(deg(90)), location=translate(STEP*2,STEP,0))
a4 = simulation.add_body(model.rotateX(deg(90)).rotateZ(deg(90)), location=translate(STEP*3,STEP,0))
a5 = simulation.add_body(model.rotateY(deg(90)).rotateZ(deg(90)), location=translate(STEP*4,STEP,0))
a6 = simulation.add_body(model.rotateY(deg(90)).rotateZ(deg(90)).rotateX(deg(90)), location=translate(STEP*5,STEP,0))
print()
a1 = simulation.add_body(model, translate(0,0,0))
a2 = simulation.add_body(model.rotateY(deg(90)), location=translate(STEP,0,0))
a3 = simulation.add_body(model.rotateX(deg(90)), location=translate(STEP*2,0,0))
a4 = simulation.add_body(model.rotateX(deg(90)).rotateZ(deg(90)), location=translate(STEP*3,0,0))
a5 = simulation.add_body(model.rotateY(deg(90)).rotateZ(deg(90)), location=translate(STEP*4,0,0))
a6 = simulation.add_body(model.rotateY(deg(90)).rotateZ(deg(90)).rotateX(deg(90)), location=translate(STEP*5,0,0))

a1.set_velocity(lin=(0,0,0), ang=(SV,S,SV))
a2.set_velocity(lin=(0,0,0), ang=(SV,S,SV))
a3.set_velocity(lin=(0,0,0), ang=(SV,SV,S))
a4.set_velocity(lin=(0,0,0), ang=(SV,SV,S))
a5.set_velocity(lin=(0,0,0), ang=(S,SV,SV))
a6.set_velocity(lin=(0,0,0), ang=(S,SV,SV))


a=True
starttime=time.time()
def animate(wdg):
	global a
	curtime = time.time()
	simulation.step()

show(animate=animate)





p.disconnect()
