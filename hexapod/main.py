#!/usr/bin/env python3

from zencad import *
import zencad.libs.kinematic
from zencad.libs.screw import screw
import zencad.malgo

import math
import time

reference = assemble.unit()


class leg0(assemble.unit):
	z = 13
	y = 3
	x = 3

	def __init__(self):
		super().__init__()
		m = box(self.x, self.y , self.z, center=True).up(self.z/2)
		self.add_shape(m)
		rot = assemble.rotator(ax=(0,1,0), parent=self, location=up(self.z))
		rot.add_triedron()
		self.rot=rot

class leg1(assemble.unit):
	z = 15
	y = 3
	x = 3

	def __init__(self):
		super().__init__()
		m = box(self.x, self.y , self.z, center=True).up(self.z/2)
		self.add_shape(m)
		self.add_shape(sphere(3).up(self.z))

		self.output = assemble.unit(location=up(self.z), parent=self)

class body(assemble.unit):
	h = 7
	r = 20

	def __init__(self):
		super().__init__()
		m = ngon(n=6, r=self.r).scaleX(0.6)
		verts = m.vertices()
		m = ngon(n=6, r=self.r).scaleX(0.6).extrude(self.h)

		self.add(m)

		self.rots = [assemble.rotator(ax=(0,0,1), parent=self, location=translate(*v.up(self.h/2).unlazy())) for v in verts]

		self.rots2=[]
		for r in self.rots:
			self.rots2.append(assemble.rotator(ax=(0,1,0), parent=r.output, location=nulltrans()))

		for r in self.rots + self.rots2:
			r.add_triedron()


class hexapod:
	def __init__(self):
		self.body = body()	
		self.legs0 = []
		self.legs1 = [] 
		
		for r in self.body.rots2:
			p0 = leg0()
			p1 = leg1()
			r.output.link(p0)
			p0.rot.link(p1)
			self.legs0.append(p0)
			self.legs1.append(p1)

		self.rots0 = self.body.rots 
		self.rots1 = self.body.rots2
		self.rots2 = [ l.rot for l in self.legs0 ]
		self.outs = [ l.output for l in self.legs1 ]
		
		for r in self.rots1[:3]: r.set_coord(deg(-80))
		for r in self.rots1[3:]: r.set_coord(deg(80))
		for r in self.rots2[:3]: r.set_coord(deg(-80))
		for r in self.rots2[3:]: r.set_coord(deg(80))

		self.chains = []
		for i in range(6):
			self.chains.append(zencad.libs.kinematic.kinematic_chain(finallink=self.legs1[i].output))

		self.typical_targets = [nulltrans()] * 6
		self.typical_targets[0] = translate(-25,0,0)
		self.typical_targets[1] = translate(-20,-20,0)
		self.typical_targets[2] = translate(-20,20,0)
		self.typical_targets[3] = translate(20,-20,0)
		self.typical_targets[4] = translate(20,20,0)
		self.typical_targets[5] = translate(25,0,0)

		self.speeds = [None] * 6

		self.group1 = [0,3,4]
		self.group2 = [1,2,5]

		self.lastposes = [ o.global_location for o in self.outs ]
		self.connected = [False] * 6

		self.leg_stress = [0] * 6 
		self.position_stress = [0] * 6 
		self.leg_stimul = [0] * 6 

		self.body_control = screw()
		self.legs_control = [vector3(0,0,0)] * 6
		self.body_target = up(10)
		self.zdiff_integral = 0

		self.ttargets = [vector3(0,0,0)] * 6

		self.in_reposition = [True] * 6
		self.need_reposition = [True] * 6
		self.on_ground = [False] * 6

		self.lastgroup = None

	def eval_body_control(self, delta, t):
		#self.body_control = screw.from_trans(self.body.global_location.inverse() * self.body_target)
		z = self.body.global_location.translation().z
		zdiff = math.sin(t/ 3) * 5 + 7 - z

		self.zdiff_integral += zdiff * delta

		zsig = zdiff * 1 + self.zdiff_integral * 0.3

		self.body_control = screw(lin=(0,12,zsig), ang=(0,0,0.1))

	def eval_legs_control(self, delta, t):
		STEP_TIME = 0.4
		MAX_STRESS = 30
		PRECONTROL = 2

		body_frame = self.body.global_location

		for i in range(6):
			self.on_ground[i] = self.legs1[i].output.global_location.translation().z <= 0.1
		
		for i in range(6):
			pos = ( self.body.global_location.inverse() * self.legs1[i].output.global_location).translation()
			v = pos - self.typical_targets[i].translation()
			v.z = 0
			self.position_stress[i] = (v).length()
		
		for i in range(6):
			self.need_reposition[i] = self.position_stress[i] > 5


		group_stress2 = 0
		group_stress1 = 0

		for g in self.group1:
			if self.position_stress[g] > group_stress1:
				group_stress1 = self.position_stress[g]

		for g in self.group2:
			if self.position_stress[g] > group_stress2:
				group_stress2 = self.position_stress[g]


		need_finalize= False

		if self.in_reposition[self.group1[0]]:
			need_finalize = abs(group_stress2) > MAX_STRESS

		elif self.in_reposition[self.group2[0]]:
			need_finalize = abs(group_stress1) > MAX_STRESS


		for i in range(6):
			if (self.leg_stress[i] > STEP_TIME or need_finalize) and self.on_ground[i] is True:
				self.in_reposition[i] = False
				self.leg_stress[i] = 0
			
		for i in range(6):
			if self.on_ground[i] is False:
				self.leg_stress[i] += delta

		for i in range(6):
			if self.need_reposition[i] and self.in_reposition[i] is False:
				self.in_reposition[i] = True
				
				if i in self.group1:		
					if group_stress2 <= group_stress1 and self.lastgroup != 1:	
						for g in self.group2:
							if self.on_ground[g] is False or self.in_reposition[g]:
								self.in_reposition[i] = False

						if self.in_reposition[i]:	
							for g in self.group1:
								self.in_reposition[g] = True
							self.lastgroup = 1
					else:
						self.in_reposition[i] = False

				elif i in self.group2:		
					if group_stress1 <= group_stress2 and self.lastgroup != 2:			
						for g in self.group1:
							if self.on_ground[g] is False or self.in_reposition[g]:
								self.in_reposition[i] = False

						if self.in_reposition[i]:	
							for g in self.group2:
								self.in_reposition[g] = True
							self.lastgroup = 2
					else:
						self.in_reposition[i] = False


		print("lgroup", self.lastgroup)
		#print("need", self.need_reposition)
		#print("in", self.in_reposition)

		for i in range(6):
			self.connected[i] = self.on_ground[i] and self.in_reposition[i] is False


		for i in range(6):
			ctr = vector3(0,0,0)
		#	in_air = not self.connected[i]
		#	if self.position_stress[i] > 10: in_air = True

			if self.in_reposition[i] and (need_finalize or self.leg_stress[i] > STEP_TIME) and not self.on_ground[i]:
				ctr += vector3(0,0,-16) 

			if self.in_reposition[i]:
				target= (self.typical_targets[i]).translation()
				current= (body_frame.inverse() * self.legs1[i].output.global_location).translation()
				diff = (target - current) * 1
				diff.z = 0
				ctr += diff

				z = self.legs1[i].output.global_location.translation().z

				ctr += vector3(0,0,5-z) * 2

			if not self.in_reposition[i] and self.on_ground[i]:
				arm = (self.body.global_location.inverse() * self.outs[i].global_location).translation()
				c = self.body_control.kinematic_carry(arm).lin
				#print(c)
				#c.z = 0
				ctr -= c

			if self.in_reposition[i] and not self.on_ground[i]:
				arm = (self.body.global_location.inverse() * self.outs[i].global_location).translation()
				c = self.body_control.kinematic_carry(arm).lin
				c.z = 0
				ctr += c * PRECONTROL
			
			if not self.in_reposition[i]:
				z = self.legs1[i].output.global_location.translation().z
				ctr += vector3(0,0,-z) * 10




		
		#	#ctr += vector3(0,0,1) * self.leg_stimul[i]
#
		#	if self.position_stress[i] > 10:
		#		ctr += vector3(0,0,10)
		#	else:
		#		ctr += vector3(0,0,-1) * self.leg_stress[i]
#
		#	print(self.position_stress)
#
#
		#	if in_air:
		#		target= (self.typical_targets[i]).translation()
		#		current= (body_frame.inverse() * self.legs1[i].output.global_location).translation()
		#		diff = (target - current) * 0.5
#
		#		ctr += diff
		#	else:
		#		z = self.legs1[i].output.global_location.translation().z
		#		ctr += vector3(0,0,-z) * 10
#
#
#
#
		#	if not in_air:
		#		arm = (self.body.global_location.inverse() * self.outs[i].global_location).translation()
		#		ctr -= self.body_control.kinematic_carry(arm).lin
		#	else:
		#		arm = (self.body.global_location.inverse() * self.outs[i].global_location).translation()
		#		cx = self.body_control.kinematic_carry(arm).lin
		#		cx.z = 0
		#		ctr += cx * 1.5


			self.legs_control[i] = ctr


	def eval_speeds(self):
		for i in range(6):
			senses = self.chains[i].sensivity(basis = self.body)
			body_frame = self.body.global_location

			#target= (self.targets[i]).translation()
			#current= (body_frame.inverse() * self.legs1[i].output.global_location).translation()


			ttarget = vector3(0,0,0)
			#ttarget += (target - current) * 0.1
			ttarget += self.legs_control[i] #- self.body_control.lin * 1

			self.ttargets[i] = ttarget

			#print("senses", senses)
			#print("target", target)

			self.speeds[i], iterations = zencad.malgo.kinematic_backpack_translation_only(
				senses=senses, 
				target=ttarget)

			#print(self.speeds[i])

			#print(senses)
			#exit()

	def apply_speeds(self, delta):
		T = 1
		for i in range(6):
			self.chains[i].apply(speeds=self.speeds[i] * 1/T, delta=delta)	

	def eval_body_position(self, delta):
		#transes = [ self.legs1[i].output.global_location.translation() for i in range(6) ]
		#self.lastposes = [ o.global_location for o in self.outs ]

		moved_speed = []
		moved_speed_arms = []
		for i in range(6):
			if self.connected[i]:
				arm = (self.body.global_location.inverse() * self.outs[i].global_location).translation()
				
				#last = self.body.global_location.inverse() * self.lastposes[i]
				#cur  = self.body.global_location.inverse() * self.outs[i].global_location

				#speed = screw.from_trans(last.inverse() * cur)
				speed = self.ttargets[i] * delta
				#speed = speed.lin
				speed.z = 0

				moved_speed.append(speed)
				arm.z = 0
				moved_speed_arms.append(arm)

		body_linear_speed = vector3(0,0,0)
		body_rotor_speed = 0

		for m in moved_speed:
			body_linear_speed -= m		
		
		if len(moved_speed) > 0:
			body_linear_speed = body_linear_speed * (1/len(moved_speed))
			
		for m, arm in zip(moved_speed, moved_speed_arms):
			diff = (m - body_linear_speed) 
			a = diff.cross(arm.normalize()).z
			l = arm.length()
			#print(a)
			body_rotor_speed += a / l
			#body_rotor_speed = 0

		if len(moved_speed) > 0:
			body_rotor_speed = body_rotor_speed * (1/len(moved_speed))


		if len(moved_speed) != 0:
			body_delta = screw(lin=body_linear_speed, ang=vector3(0,0,body_rotor_speed)).to_trans()
			self.body.relocate(self.body.global_location * body_delta) 

		for i in range(6):
			self.lastposes[i] = self.outs[i].global_location


	def grounding(self):
		body_z = self.body.global_location.translation().z

		minz = 10000000 
		for o in self.outs:
			if o.global_location.translation().z < minz:
				minz = o.global_location.translation().z

		zmove = min(body_z, minz)

		self.body.moveZ(-zmove)

hexapod = hexapod()

hexapod.body.relocate(translate(40,50) * rotateZ(deg(30)))


start_time = time.time()
lasttime = start_time
def animate(wdg):
	global lasttime
	curtime = time.time()
	t = curtime - start_time
	delta = curtime-lasttime
	lasttime = curtime

	if delta > 0.1:
		delta = 0.05

	if t < 1:
		return

	hexapod.eval_body_control(delta, time.time())
	hexapod.eval_legs_control(delta, time.time())
	hexapod.eval_speeds()	
	hexapod.apply_speeds(delta)	
	hexapod.grounding()
	hexapod.body.location_update()
	hexapod.eval_body_position(delta)
	hexapod.body.location_update()

	eye = point3(*hexapod.body.global_location.translation())
	center = point3(*hexapod.body.global_location.translation())
	eye.x = -200
	eye.z = 00

	center.z = 0

	#wdg.view.set_center(center)
	#wdg.view.set_eye(eye)

disp(hexapod.body)
show(animate=animate)
