#!/usr/bin/env python3

import time
import random

from zencad import *
import zencad.assemble


BALL_POSITION = [0,0]
BALL_SPEED_NORMAL = math.sqrt(100**2 * 2)
BALL_SPEED = [100,100]
BOX_WIDTH, BOX_LENGTH=300,300


def change_angle():
	global BALL_SPEED

	angle = math.atan2(BALL_SPEED[1], BALL_SPEED[0])
	angle += random.uniform(-0.2, 0.2)


	BALL_SPEED = [math.cos(angle) * BALL_SPEED_NORMAL, math.sin(angle) * BALL_SPEED_NORMAL]

class car(zencad.assemble.unit):
	size=(28.5,30,10)
	wr = 6
	wh = 3
	s = 2
	def __init__(self):
		super().__init__()
		size = self.size
		s = self.s
		c = cylinder(r=self.wr, h=self.wh)
		b = box(size=self.size,center=True)
		l = loft([rectangle(30,10,center=True,wire=True), rectangle(20,5,center=True,wire=True).up(5)])

		b = b + multitrans([
			move(size[0]/2, -size[1]/3, -s) * rotateY(deg(90)),
			move(size[0]/2, size[1]/3, -s) * rotateY(deg(90)),
			move(-size[0]/2, -size[1]/3, -s) * rotateY(-deg(90)),
			move(-size[0]/2, size[1]/3, -s) * rotateY(-deg(90)),
		])(c) + l.rotateX(-deg(90)).moveY(15)

		self.add_shape(b.up(self.wr+s))

car = car()

disp(box(300,300,1, center=True).down(0.5))
disp(car)
ball = disp(sphere(10), color=zencad.color.red)

target = translate(150,150,0)

stime = time.time()
lasttime = stime
inited= False
def animate(wdg):
	global BALL_POSITION
	global inited
	global lasttime
	curtime = time.time()
	delta = curtime - lasttime
	DELTA= delta
	lasttime = curtime

	target = translate(*BALL_POSITION)

	if not inited:
		inited = True
		return

	error = car.global_location.inverse() * target
	error_translation = error.translation()

	KV = 1.5
	KW = 0.04
	V = error_translation[1] * KV
	W = -error_translation[0] * KW
	car.relocate(car.global_location * translate(0, delta * V, 0) * rotateZ(delta*W))

	BALL_POSITION[0] += BALL_SPEED[0] * DELTA 
	BALL_POSITION[1] += BALL_SPEED[1] * DELTA

	if BALL_POSITION[0] > BOX_WIDTH/2:
		BALL_SPEED[0] = - BALL_SPEED[0]
		BALL_POSITION[0] = BOX_WIDTH/2
		change_angle()

	if BALL_POSITION[0] < -BOX_WIDTH/2:
		BALL_SPEED[0] = - BALL_SPEED[0]
		BALL_POSITION[0] = -BOX_WIDTH/2
		change_angle()

	if BALL_POSITION[1] > BOX_LENGTH/2:
		BALL_SPEED[1] = - BALL_SPEED[1]
		BALL_POSITION[1] = BOX_LENGTH/2
		change_angle()

	if BALL_POSITION[1] < -BOX_LENGTH/2:
		BALL_SPEED[1] = - BALL_SPEED[1]
		BALL_POSITION[1] = -BOX_LENGTH/2
		change_angle()

	ball.relocate(translate(BALL_POSITION[0], BALL_POSITION[1]))

show(animate = animate)

