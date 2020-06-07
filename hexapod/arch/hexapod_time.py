#!/usr/bin/env python3

from zencad import *
import zencad.libs.kinematic
from zencad.libs.screw import screw
import zencad.malgo

import zencad.libs.bullet

import math
import time
import pybullet

import threading

TIMESTEP = 1/240

reference = assemble.unit()
simulation = zencad.bullet.simulation(
	gravity=(0,0,-50), plane=True, gui=False, scale_factor=10, time_step=TIMESTEP, substeps=1)

class leg_controller:
	def __init__(self, hexapod, index, reference):
		self.hexapod = hexapod
		self.flink = hexapod.outs[index]
		self.slink = hexapod.body
		self.chain = zencad.libs.kinematic.kinematic_chain(self.flink, self.slink)
		self.reference_position = vector3(reference)
		self.grounded_target_position=self.reference_position + vector3(0,0,-10)
		self.target_position = self.reference_position

		self.bcmode = True
		self.control_mode = "velocity"

		self.velocity = vector3(0,0,0)

		self.servos = [
			self.hexapod.rots0_servos[index],
			self.hexapod.rots1_servos[index],
			self.hexapod.rots2_servos[index]
		]

		T1=1
		T2=3
		KSI1=4
		KSI2=1
		M=1
		F=0.2

		#self.servos[0].set_regs(
		#	pidspd=zencad.bullet.pid.by_params(T=T1, ksi=KSI1, M=M),
		#	pidpos=zencad.bullet.pid.by_params(T=T2, ksi=KSI2, M=1), 
		#	filt=F)
		#self.servos[1].set_regs(
		#	pidspd=zencad.bullet.pid.by_params(T=T1, ksi=KSI1, M=M),
		#	pidpos=zencad.bullet.pid.by_params(T=T2, ksi=KSI2, M=1), 
		#	filt=F)
		#self.servos[2].set_regs(
		#	pidspd=zencad.bullet.pid.by_params(T=T1, ksi=KSI1, M=M),
		#	pidpos=zencad.bullet.pid.by_params(T=T2, ksi=KSI2, M=1),
		#	filt=F)

		#KP01=100
		#KP02=33
		#KP11=100
		#KP12=33
		#KP21=100
		#KP22=33

		KP01=80
		KP02=30
		KP11=115
		KP12=35
		KP21=140
		KP22=40

		self.servos[0].set_regs(
			pidspd=zencad.bullet.pid(kp=KP01,ki=0),
			pidpos=zencad.bullet.pid(kp=KP02,ki=0), 
			filt=F)
		self.servos[1].set_regs(
			pidspd=zencad.bullet.pid(kp=KP11,ki=0),
			pidpos=zencad.bullet.pid(kp=KP12,ki=0), 
			filt=F)
		self.servos[2].set_regs(
			pidspd=zencad.bullet.pid(kp=KP21,ki=0),
			pidpos=zencad.bullet.pid(kp=KP22,ki=0),
			filt=F)
#
		#KI0 = 10
		#KIF0=0
		#KPOS0 = 0
		#KP0 = 16
#
		#KI1 = 10
		#KIF1=0
		#KPOS1 = 0
		#KP1 = 8
#
		#KI2 = 0
		#KIF2=0
		#KPOS2 = 0
		#KP2 = 50

		#self.servos[0].set_koeffs(kpos=KPOS0, kspd_i=KI0, kspd=KP0, kspd_if=KIF0, maxforce=100000)
		#self.servos[1].set_koeffs(kpos=KPOS1, kspd_i=KI1, kspd=KP1, kspd_if=KIF1, maxforce=100000)
		#self.servos[2].set_koeffs(kpos=KPOS2, kspd_i=KI2, kspd=KP2, kspd_if=KIF2, maxforce=100000)

	def is_grounded(self):
		#info = pybullet.getContactPoints(
		#	hexapod.simulation_controller.boxId, simulation.plane_index, 
		#	self.flink.current_index2, -1)
#
		#for i in info:
		#	print("DIST:", i[8])
		#	if i[8] < 0.00001:
		#		return True
		#print(self.flink.global_location.translation().z)

		return self.flink.global_location.translation().z < 5

	def stress(self):
		arm = ((self.hexapod.body.global_location.inverse() 
			* self.flink.global_location).translation())
		vec = (arm - self.reference_position)
		vec.z = 0
		return vec.length()


	def set_body_mirror_control(self, sig, koeff=1, z=None, rkoeff=0):
		self.control_mode = "velocity"
		#sig = sig * koeff
		arm = ((self.hexapod.body.global_location.inverse() 
			* self.flink.global_location).translation())
		c = sig.kinematic_carry(arm).lin

		referr = self.reference_position - arm
		zerr = arm.z
		#print(zerr)
		if z is not None:
			z_korrection = vector3(0,0,zerr)
		else:
			z_korrection = vector3(0,0,0)

		ressig = -c * koeff #+ referr*rkoeff + z_korrection*1.2

		#print(ressig)
		self.set_velocity(ressig)		
	
	def set_grounded_target(self):
		self.control_mode = "position"
		self.target_position = self.grounded_target_position

	def set_position_target(self, tgt):
		self.control_mode = "position"
		self.target_position = vector3(tgt)

	def go_pre_position(self):
		self.control_mode = "position"
		self.target_position = self.reference_position + vector3(0,10,0) + vector3(0,13,0)
		self.bcmode = False	

	def go_home_position(self):
		self.control_mode = "position"
		self.target_position = self.reference_position
		self.bcmode = False	

	def go_ground(self):
		self.control_mode = "position"
		z = hexapod.body.global_location.translation().z
		self.target_position = self.reference_position + vector3(0,0,-z) + vector3(0,13,0)
		self.bcmode = False	

	def set_velocity(self, vec):
		self.velocity = vector3(vec)

	def current_position(self):
		opos = self.hexapod.body.global_location.inverse() * self.flink.global_location
		otgt = opos.translation()
		return otgt

	def eval_position_control(self):
		K = 4
		error = self.target_position - self.current_position()
		sig = error * K
		self.set_velocity(sig)


	def serve(self, delta):
		#self.set_velocity((0,0,10))

		a=time.time()
		if self.control_mode=="position":
			self.eval_position_control()		
		elif self.control_mode=="velocity":
			pass
			#self.eval_velocity_control()

		b=time.time()
		sigs = self.chain.decompose_linear(self.velocity, use_base_frame=True)

		c=time.time()
		#print(sigs)

		self.servos[0].set_speed2(sigs[0])
		self.servos[1].set_speed2(sigs[1])
		self.servos[2].set_speed2(sigs[2])
		d=time.time()

		#print(b-a, c-b, d-c)

class leg0(assemble.unit):
	z = 26
	y = 2
	x = 1

	def __init__(self):
		super().__init__()
		m = box(self.x, self.y , self.z, center=True).up(self.z/2)
		self.add_shape(m)
		rot = assemble.rotator(axis=(0,1,0), parent=self, location=up(self.z))
		rot.add_triedron()
		self.rot=rot

class leg1(assemble.unit):
	z = 30
	y = 2
	x = 1

	def __init__(self):
		super().__init__()
		m = box(self.x, self.y , self.z, center=True).up(self.z/2)
		self.add_shape(m)

		m = sphere(1) 
		self.output = assemble.unit([m], location=up(self.z), parent=self)

class body(assemble.unit):
	h = 3
	r = 30

	def __init__(self):
		super().__init__()
		m = ngon(n=6, r=self.r).scaleX(0.6)
		verts = m.vertices()
		m = ngon(n=6, r=self.r).scaleX(0.6).extrude(self.h)

		self.add(m)

		self.rots = [assemble.rotator(axis=(0,0,1), parent=self, location=translate(*v.up(self.h/2).unlazy())) for v in verts]

		self.rots2=[]
		for r in self.rots:
			self.rots2.append(assemble.rotator(axis=(0,1,0), parent=r.output, location=nulltrans()))

		for r in self.rots + self.rots2:
			r.add_triedron()


class hexapod:
	def __init__(self):
		self.body = body()	
		self.legs0 = []
		self.legs1 = [] 
		self.body_position_error_integral = screw()
		self.body_control_target = translate(0,0,10)
		self.errspd = screw()
		self.last_corrected_group = -1
		self.last_on_ground = False
		
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
		
		for r in self.rots1[:3]: r.set_coord(deg(-50))
		for r in self.rots1[3:]: r.set_coord(deg(50))
		for r in self.rots2[:3]: r.set_coord(deg(-50))
		for r in self.rots2[3:]: r.set_coord(deg(50))

		self.rots0_servos = [ zencad.bullet.servo_controller3(simulation=simulation, kunit=self.rots0[i]) for i in range(len(self.rots0)) ]
		self.rots1_servos = [ zencad.bullet.servo_controller3(simulation=simulation, kunit=self.rots1[i]) for i in range(len(self.rots1)) ]
		self.rots2_servos = [ zencad.bullet.servo_controller3(simulation=simulation, kunit=self.rots2[i]) for i in range(len(self.rots2)) ]

		self.rots_all = self.rots0_servos + self.rots1_servos + self.rots2_servos
		#for r in self.rots_all: r.set_koeffs(ki=100, kp=30)

		self.init_leg_controllers()

	def init_servos(self):
		for r in self.rots_all:
			r.init()

	def init_leg_controllers(self):
		A = 40
		B = 27
		C = 37
		Z = 0
		self.leg_controllers = [
			leg_controller(self, 0, (-A,0,Z)),
			leg_controller(self, 1, (-B,-C,Z)),
			leg_controller(self, 2, (-B,C,Z)),
			leg_controller(self, 3, (B,-C,Z)),
			leg_controller(self, 4, (B,C,Z)),
			leg_controller(self, 5, (A,0,Z))
		]

		self.left_ctrs=[self.leg_controllers[2], self.leg_controllers[0], self.leg_controllers[1]]
		self.right_ctrs=[self.leg_controllers[4], self.leg_controllers[5], self.leg_controllers[3]]
		
		self.group1_ctrs=[self.leg_controllers[2], self.leg_controllers[5], self.leg_controllers[1]]
		self.group2_ctrs=[self.leg_controllers[4], self.leg_controllers[0], self.leg_controllers[3]]

	def has_contact_with_ground(self):
		ret = []
		for i in range(hexapod.simulation_controller.links_number):
			r = pybullet.getContactPoints(hexapod.simulation_controller.boxId, simulation.plane_index, i, -1)
			if r != ():
				ret.append(r)
		return len(ret) > 0

	def group1_activate_correction(self):
		for s in self.group1_ctrs:
			s.bcmode = False
		self.last_corrected_group = 1
		self.corrected_start = time.time()

	def group2_activate_correction(self):
		for s in self.group2_ctrs:
			s.bcmode = False
		self.last_corrected_group = 2
		self.corrected_start = time.time()

	def activate_body_control0(self):
		STRESS_TRIGGER = 0
		CORRECTION_TIME = 3
		target = self.body_control_target
		current = self.body.global_location

		vel = self.simulation_controller.velocity()
		velscr = screw(ang=vel[1], lin=vel[0])
		#print(velscr)

		error = current.inverse() * target
		errsig = screw.from_trans(error)

		#alpha = 1
		self.errspd = -velscr 
		#if error.translation().length() < 5:	
		#	K = 2
		#	KI = 0.002
		#	KD = 0.1
		#	self.body_position_error_integral += errsig
		#else:
		K = 0.6
		KI = 0
		
		KD = 0.2
		self.body_position_error_integral=screw()
		errsig = errsig * K + self.body_position_error_integral * KI + self.errspd * KD

		for i, s in enumerate(self.leg_controllers):
			if s.bcmode:
				s.set_body_mirror_control(errsig, koeff=1, z=None, rkoeff=0)


	def body_control(self):
		K = 1.3
		KI = 0
		KD = 0#10
		STRESS_TRIGGER = 0
		CORRECTION_TIME = 3
		target = self.body_control_target
		current = self.body.global_location

		vel = self.simulation_controller.velocity()
		velscr = screw(ang=vel[1], lin=vel[0])
		#print(velscr)

		error = current.inverse() * target
		errsig = screw.from_trans(error)

		alpha = 0.1
		self.errspd = self.errspd*(1-alpha) + (-velscr + self.errspd) * alpha
		self.body_position_error_integral += errsig
		errsig = errsig * K + self.body_position_error_integral * KI + self.errspd * KD

		group1_correction_mode = False
		group2_correction_mode = False
		
		for s in self.group1_ctrs:
			if s.bcmode == False:
				group1_correction_mode = True
		
		for s in self.group2_ctrs:
			if s.bcmode == False:
				group2_correction_mode = True
		
		if group1_correction_mode is False and group2_correction_mode is False:
			if self.last_corrected_group != 1:
				self.last_corrected_group = 1
				print("activete 1 correction")
				for s in self.group1_ctrs:
					stress = s.stress()
					if stress > STRESS_TRIGGER:
						self.group1_activate_correction()
		
			elif self.last_corrected_group != 2: 
				self.last_corrected_group = 2
				print("activete 2 correction")
				for s in self.group2_ctrs:
					stress = s.stress()
					if stress > STRESS_TRIGGER:
						self.group2_activate_correction()

		if group1_correction_mode is True and time.time() - self.corrected_start > CORRECTION_TIME:
			print("group1_correction_mode is True")
			leave = True
			for s in self.group1_ctrs:
				if not s.is_grounded():
					leave = False
				else:
					print("GROUNDED")
					s.bcmode=True
			if leave:
				group1_correction_mode=False


		if group2_correction_mode is True and time.time() - self.corrected_start > CORRECTION_TIME:
			print("group2_correction_mode is True")
			leave = True
			for s in self.group2_ctrs:
				if not s.is_grounded():
					leave = False
				else:
					s.bcmode=True
			if leave:
				group2_correction_mode=False



		print(self.last_corrected_group)

		for i, s in enumerate(self.leg_controllers):
			if s.bcmode:
				s.set_body_mirror_control(errsig, koeff=1, z=None, rkoeff=0)
			else:
				if time.time() - self.corrected_start < CORRECTION_TIME:
					s.set_body_mirror_control(errsig, koeff=1, rkoeff=1, z=0)
				else:
					s.set_body_mirror_control(errsig, koeff=-1, rkoeff=0, z=None)
				

	def body_trajectory_control(self, delta):
		#pass
		#if time.time() - self.grounded_time > 1.5:
		self.body_control_target = self.body_control_target * screw(lin=(0,12,0), ang=(0,0,0)).scale(delta).to_trans()

	def activate_home_program(self):
		self.left_ctrs[0].set_position_target((-25,40,0))
		self.left_ctrs[1].set_position_target((-35,0,0))
		self.left_ctrs[2].set_position_target((-25,-40,0))
		
		self.right_ctrs[0].set_position_target((25,40,0))
		self.right_ctrs[1].set_position_target((35,0,0))
		self.right_ctrs[2].set_position_target((25,-40,0))

	def serve(self, state, iteration):
		#on_ground = not self.has_contact_with_ground()
		#if on_ground and self.last_on_ground == False:
		#	self.grounded_time = time.time()
		#
		#if on_ground:
		#	for s in self.leg_controllers:
		#		s.set_grounded_target()
		#
		#else:
		#	self.body_trajectory_control(state.delta)
		#	self.body_control()


		#self.left_ctrs[1].servos[1].set_position_target(1)
		#self.left_ctrs[1].servos[1].set_speed_target(1)
		#print(self.left_ctrs[1].servos[1].speed())

		if iteration%8 ==0:
			a = time.time()
			for s in self.leg_controllers:
				s.serve(state.delta)

		b = time.time()
		for r in self.rots_all:
			r.serve(state.delta)
		
		c = time.time()
			#r.serve_spd_only(state.delta)

		#self.last_on_ground = on_ground


hexapod = hexapod()
hexapod.body.relocate(translate(0,0,0))

hexapod.simulation_controller = simulation.add_assemble(hexapod.body, fixed_base=False)
hexapod.init_servos()


T1=1 # Коррекция
T2=0.6 # Опускание
T12=0.7 # совместное управление

state = zencad.animate.AnimationState(None)
state.start_time = 0
state.timestamp(0)

TOKEN = False

def foo(iteration):

	u = time.time()
	if state.loctime < 1:
		hexapod.activate_home_program()

	elif state.loctime < 3:
		for s in hexapod.leg_controllers:
			s.bcmode=True
		hexapod.activate_body_control0()
	
	else:
		ltime = (state.loctime - 5) % (T1*2 + T2*2 + 2*T12)
		if ltime < T1:
			for c in hexapod.group1_ctrs: c.bcmode = True
			for c in hexapod.group2_ctrs: c.go_pre_position()
			hexapod.activate_body_control0()
		
		elif ltime < T2+T1:
			for c in hexapod.group1_ctrs: c.bcmode = True
			for c in hexapod.group2_ctrs: c.go_ground()
			hexapod.activate_body_control0()
	
		elif ltime < T1+T2+T12:
			for c in hexapod.group1_ctrs: c.bcmode = True
			for c in hexapod.group2_ctrs: c.bcmode = True
			hexapod.activate_body_control0()
		
		elif ltime < T1*2+T2+T12:
			for c in hexapod.group1_ctrs: c.go_pre_position()
			for c in hexapod.group2_ctrs: c.bcmode = True
			hexapod.activate_body_control0()
		
		elif ltime < T2*2+T1*2+T12:
			for c in hexapod.group1_ctrs: c.go_ground()
			for c in hexapod.group2_ctrs: c.bcmode = True
			hexapod.activate_body_control0()
	
		elif ltime < T1*2+T2*2+T12*2:
			for c in hexapod.group1_ctrs: c.bcmode = True
			for c in hexapod.group2_ctrs: c.bcmode = True
			hexapod.activate_body_control0()
	
		hexapod.body_trajectory_control(state.delta)

	a = time.time()
	if iteration % 1 == 0:
		hexapod.serve(state, iteration)

	b = time.time()
	simulation.step()

	c = time.time()

	print(a-u, b-a, c-b)

def bar():
	iteration = 0
	while True:
		iteration += 1
		if TOKEN:
			return
		state.timestamp(state.time + TIMESTEP)
		foo(iteration)

		print(state.time)

thr = threading.Thread(target=bar)
thr.start()

def close_handle():
	global TOKEN 
	TOKEN = True

def animate(state):
	pass

show(animate=animate, close_handle=close_handle)
