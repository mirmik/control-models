#!/usr/bin/env python3

import pybullet as p
import time
import pybullet_data

import zencad
from zencad import *
from zencad.libs.obj_writer import write_as_obj_wavefront

import tempfile

from micar import MODEL
#from kran import MODEL

@zencad.lazifier.lazy.file_creator("path")
def volumed_colision(inpath, path, logpath, alpha, resolution):
	p.vhacd(inpath, path, logpath,resolution=resolution) 
	#	convexhullApproximation=0,
	#	mode = 0,
	#	convexhullDownsampling=16,
	#	planeDownsampling=16,
	#	depth=32,
	#	alpha = 0.0005,
	#	beta = 0.0005,
	#	gamma= 0.0005 )

class pybullet_shape_bind:
	def __init__(self, shape, location=nulltrans(),  mass=None, modelfile=None, non_convex_collision=False, alpha=0.05, resolution=100000):
		self.shape = shape
		self.alpha = alpha
		self.resolution = resolution
		self.global_damping = 0

		if mass is None:
			mass = shape.mass()

		inertia_axes = shape.principal_inertia_axes()
		inertia_frame = shape.inertia_frame()
		
		self.masscenter = evalcache.unlazy_if_need(shape.center())
		self.inertia_orientation = inertia_frame.rotation()
		self.inertia_orientation = self.inertia_orientation.x.unlazy(), self.inertia_orientation.y.unlazy(), self.inertia_orientation.z.unlazy(), self.inertia_orientation.w.unlazy() 
		
		nodes, triangles = zencad.triangulate(shape, 0.01)
		if modelfile is None:
			self.meshpath = shape.__lazyhexhash__[12:] + ".obj"
		else:
			self.meshpath = modelfile
		write_as_obj_wavefront(self.meshpath, nodes, triangles)

		if non_convex_collision:
			self.meshpath2 = self.meshpath + ".convex.obj"
			volumed_colision(self.meshpath, self.meshpath2, self.meshpath + ".log.txt", alpha, resolution)
		else:
			self.meshpath2 = self.meshpath
		
		self.visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH, fileName=self.meshpath2)
		self.collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH, fileName=self.meshpath2)
		
		#self.kkk = nulltrans()
		#self.kkk = rotateX(deg(60)) * rotateZ(deg(180))
		#self.kkk = inertia_frame.unlazy()

		#self.tquat = (rotateX(deg(60)) * rotateZ(deg(180))).rotation()
		self.boxId = p.createMultiBody(baseMass=mass,
					  baseCollisionShapeIndex=self.collisionShapeId,
					  baseVisualShapeIndex=self.visualShapeId,
					  basePosition = location.translation(),
					  baseOrientation = location.rotation(),
					  baseInertialFramePosition = self.masscenter
					  #,baseInertialFrameOrientation=self.kkk.rotation()
					  )

		p.changeDynamics(self.boxId, -1, linearDamping=self.global_damping, angularDamping=self.global_damping)
		p.changeDynamics(self.boxId, -1, maxJointVelocity=100000)

	def bind_to_scene(self, scene, color=zencad.default_color):
		self.ctr = disp(self.shape, scene=scene, color=color)
		return self

	def update(self):
		cubePos, cubeOrn = p.getBasePositionAndOrientation(self.boxId)
		rot = pyservoce.quaternion(*cubeOrn).to_transformation()
		tr = translate(*cubePos) * rot * translate(vector3(self.masscenter)).inverse()
		self.ctr.relocate(tr)

	def velocity(self):
		spd = p.getBaseVelocity(self.boxId)
		return vector3(spd[0]), vector3(spd[1])

	def set_velocity(self, lin=(0,0,0), ang=(0,0,0)):
		p.resetBaseVelocity(self.boxId, lin, ang)



class pybullet_simulation:
	def __init__(self, gravity=(0,0,0), plane=1000):
		self.client = p.connect(p.DIRECT)#or p.DIRECT for non-graphical version
		p.setRealTimeSimulation(1) 
		#p.setTimeStep(0.01) 
		self.binds = []
		self.set_gravity(gravity)

		self.library_included = False

		if plane:
			if plane is True:
				plane = 1000
			self.add_body(box(plane,plane,1, center=True).down(0.5), location=translate(0,0,0), mass=0, color=(0,0,0,1))

	def set_gravity(self, x, y=None, z=None):
		v = zencad.util.vector3(x,y,z)
		p.setGravity(*v)
		
	def add_body(self, shape, location=nulltrans(), mass=None, non_convex_collision=False, scene=zencad.default_scene, color=zencad.default_color, resolution=100000):
		bind = pybullet_shape_bind(shape, location, mass=mass,
			modelfile=shape.__lazyhexhash__[:12]+".obj", 
			non_convex_collision= non_convex_collision,
			resolution=resolution).bind_to_scene(scene,color)

		self.binds.append(bind)
		return bind

	def include_library(self):
		if self.library_included is False:
			self.library_included = True
			p.setAdditionalSearchPath(pybullet_data.getDataPath())

	def add_world_plane(self):
		self.include_library()
		self.planeId = p.loadURDF("plane.urdf")

	def step(self):
		for o in self.binds:
			o.update()
		p.stepSimulation()


simulation = pybullet_simulation(gravity=(0,0,-10000), plane=True)

mb = (box(800,400,60, center=True) - box(780,380,40, center=True).up(20)).up(20) 
m = (box(800,400,60, center=True) - box(780,380,40, center=True).up(20)).up(20) - \
	multitrans([
		translate(-375,175,0),
		translate(-375,-175,0),
		translate(375,-175,0),
		translate(375,175,0),
		translate(0,-175,0),
		translate(0,175,0)
	])(cylinder(h=80,r=50).down(20))


ppp = (m - halfspace().up(20)).solids()

simulation.add_body(mb ^ halfspace().up(20), mass=0)
for pp in ppp: simulation.add_body(pp, mass=0)

aaa = simulation.add_body(sphere(r=15), mass=2,color=color.white, location=up(40) * right(200) *forw(20))
simulation.add_body(sphere(r=15), mass=1,location=up(40) * translate(40-40,0,0))
simulation.add_body(sphere(r=15), mass=1,location=up(40) * translate(40-66,16,0))
simulation.add_body(sphere(r=15), mass=1,location=up(40) * translate(40-66,-16,0))
simulation.add_body(sphere(r=15), mass=1,location=up(40) * translate(40-92,0,0))
simulation.add_body(sphere(r=15), mass=1,location=up(40) * translate(40-92,31,0))
simulation.add_body(sphere(r=15), mass=1,location=up(40) * translate(40-92,-31,0))
simulation.add_body(sphere(r=15), mass=1,location=up(40) * translate(40-92-31,16,0))
simulation.add_body(sphere(r=15), mass=1,location=up(40) * translate(40-92-31,-16,0))
simulation.add_body(sphere(r=15), mass=1,location=up(40) * translate(40-92-31,16+31,0))
simulation.add_body(sphere(r=15), mass=1,location=up(40) * translate(40-92-31,-16-31,0))
simulation.add_body(sphere(r=15), mass=1,location=up(40) * translate(40-92-31*2,0,0))
simulation.add_body(sphere(r=15), mass=1,location=up(40) * translate(40-92-31*2,31,0))
simulation.add_body(sphere(r=15), mass=1,location=up(40) * translate(40-92-31*2,-31,0))
simulation.add_body(sphere(r=15), mass=1,location=up(40) * translate(40-92-31*2,31+31,0))
simulation.add_body(sphere(r=15), mass=1,location=up(40) * translate(40-92-31*2,-31-31,0))

#simulation.add_body(sphere(r=15), location=up(60) * forw(175) * left(375))



#simulation.add_body(box(5).rotateX(deg(60)).left(30), translate(0,0,10))
#simulation.add_body(sphere(5), translate(0,0,10))
#simulation.add_body(MODEL.scale(0.5), translate(0,0,0), non_convex_collision=True)
#simulation.add_body(MODEL.scale(0.5), translate(0,0,40) * rotateZ(deg(60)), non_convex_collision=True)
#simulation.add_body(MODEL.scale(0.5), translate(0,0,60) * rotateZ(deg(60)), non_convex_collision=True)
#simulation.add_body(box(5).rotateX(deg(60)), translate(0,0,80), mass=10000)
#obj = simulation.add_body(cylinder(r=5, h=100), translate(40,40,50) * rotateX(deg(60)))
#obj.set_velocity(lin=(0,0,0), ang=rotateX(deg(90+60))(vector3(0,0,3.14)))

#p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
#planeId = p.loadURDF("plane.urdf")


#simulation.add_body(cylinder(r=10, h=20, center=True).rotX(deg(90)), location=up(10))
#obj2=simulation.add_body(box(200,20,20, center=True), location=up(30))
#simulation.add_body(box(10,10,10, center=True), location=up(50) * right(70))
#simulation.add_body(box(20,20,20, center=True), location=up(60) * right(-70))
#obj = simulation.add_body(box(20,20,20, center=True), location=up(30) * right(-70) * forw(70))
#obj.set_velocity((0,-300,0))


a=True
starttime=time.time()
def animate(wdg):
	global a
	curtime = time.time()
	simulation.step()
	#print(obj.velocity())
	#LLL = p.calculateMassMatrix(obj2.boxId, [])

	if (time.time() - starttime > 3 and a):
		a= False
		aaa.set_velocity([-9000,-900,0])



	#pass

show(animate=animate)





p.disconnect()
