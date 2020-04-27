#!/usr/bin/env python3

import time
import random

from zencad import *
import zencad.assemble


BOX_WIDTH, BOX_LENGTH=300,300

class ball(assemble.unit):
	def __init__(self, color, stable=False):
		super().__init__()
		if not stable:
			self.BALL_POSITION = [0,0]
			self.BALL_SPEED_NORMAL = math.sqrt(100**2 * 2)
			self.BALL_SPEED = [200 * random.random() - 100, 200 * random.random() - 100]
		else:
			self.BALL_POSITION = [random.random() * BOX_LENGTH - BOX_LENGTH/2,random.random() * BOX_WIDTH - BOX_WIDTH/2]
			self.BALL_SPEED_NORMAL = 0
			self.BALL_SPEED = [0, 0]

		self.add_shape(sphere(10), color=color)

	def update(self, DELTA):

		self.BALL_POSITION[0] += self.BALL_SPEED[0] * DELTA 
		self.BALL_POSITION[1] += self.BALL_SPEED[1] * DELTA
	
		if self.BALL_POSITION[0] > BOX_WIDTH/2:
			self.BALL_SPEED[0] = - self.BALL_SPEED[0]
			self.BALL_POSITION[0] = BOX_WIDTH/2
			self.change_angle()
	
		if self.BALL_POSITION[0] < -BOX_WIDTH/2:
			self.BALL_SPEED[0] = - self.BALL_SPEED[0]
			self.BALL_POSITION[0] = -BOX_WIDTH/2
			self.change_angle()
	
		if self.BALL_POSITION[1] > BOX_LENGTH/2:
			self.BALL_SPEED[1] = - self.BALL_SPEED[1]
			self.BALL_POSITION[1] = BOX_LENGTH/2
			self.change_angle()
	
		if self.BALL_POSITION[1] < -BOX_LENGTH/2:
			self.BALL_SPEED[1] = - self.BALL_SPEED[1]
			self.BALL_POSITION[1] = -BOX_LENGTH/2
			self.change_angle()
	
		self.relocate(translate(self.BALL_POSITION[0], self.BALL_POSITION[1]))


	def change_angle(self):	
		angle = math.atan2(self.BALL_SPEED[1], self.BALL_SPEED[0])
		angle += random.uniform(-0.2, 0.2)
	
		self.BALL_SPEED = [math.cos(angle) * self.BALL_SPEED_NORMAL, math.sin(angle) * self.BALL_SPEED_NORMAL]

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

b0 = ball(zencad.color.red)

dballs = [ball(zencad.color.blue, stable=True) for i in range(10)]
balls = [b0] + dballs

disp(balls)

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

	for b in balls: b.update(delta)
	
	target = translate(*b0.BALL_POSITION)
	dangers = [translate(*db.BALL_POSITION) for db in dballs]

	if not inited:
		inited = True
		return

	error = car.global_location.inverse() * target
	error_translation = error.translation()

	error_dir = error_translation.normalize()
	error_len = error_translation.length()

	error_signal = error_dir * (error_len*5)
	
	danger_signals = [(car.global_location.inverse() * d).translation() for d in dangers]
	danger_signal_dirs = [d.normalize() for d in danger_signals]
	danger_signal_lens = [d.length() for d in danger_signals]

	print(danger_signal_lens[0])
	print("log", math.log(danger_signal_lens[0]))

	for i in range(len(danger_signals)):
		t = danger_signal_lens[i] + 0.1
		danger_signal = 0
		danger_signal += 300000 / (t*t) 
		danger_signal += 200 / t
		#danger_signal += 300 / math.log(t)

		error_signal -= danger_signal * danger_signal_dirs[i]

	KV = 1.5
	KW = 0.04
	V = error_signal[1] * KV
	W = -error_signal[0] * KW
	car.relocate(car.global_location * translate(0, delta * V, 0) * rotateZ(delta*W))


show(animate = animate)

