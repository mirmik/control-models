#!/usr/bin/env python3
#coding: utf-8

from zencad import *
#lazy.diag = True
#lazy.fastdo =True

def make_body(con, cil, centrad, tooth, forbolt):
	r1, r2, h = con
	rc, hc = cil
	toothes, tooth_radius = tooth
	rdel, rhole, deltaangle = forbolt
	t = h - hc
	
	ctr = multitrans([right(rdel), left(rdel), forw(rdel), back(rdel)])
	return (
		cone(r1=r1, r2=r2, h=h) 
		- cylinder(r=rc, h=hc).up(t) 
		- cylinder(r=centrad, h=t)
		- ctr(cylinder(r=rhole, h=t)).rotateZ(deltaangle)
		+ rotate_array(toothes)(cylinder(r=tooth_radius, h=h-t).up(t).forw(rc))
	)

def make_base_support(r1, r2, s, h, t):
	return (
		cone(r1=r1+s, r2=r2+s, h=h, yaw=deg(180))
		- halfspace().rotateY(-math.atan2(h,r2-r1)).right(r1)
		- halfspace().rotateY(deg(90)).left(t)
		- cone(r1, r2, h)
	)
	
def make_holder(t):
	polyg_xseg = 4.5
	polyg_yseg = 4.5
	hold_t = t
	
	hold_fil = 3
	hold_fil2 = 1
	eey = 2.8

	polyg = polygon([
		(polyg_xseg*0,		polyg_yseg*0.5), 
		(polyg_xseg, 		polyg_yseg*0.5),
		(polyg_xseg*2, 		polyg_yseg),
		(polyg_xseg*4, 		polyg_yseg),
		(polyg_xseg*4, 		polyg_yseg*3.5),
		(polyg_xseg*2, 		polyg_yseg*3.5),
		(polyg_xseg*0, 		polyg_yseg*3),
		(polyg_xseg*(-1),	polyg_yseg*2),
		(polyg_xseg*(-1),	polyg_yseg*1),
	]).left(polyg_xseg*3)
	
	polyg = fillet(polyg, hold_fil)
	
	m = fillet(polyg.extrude(hold_t), hold_fil2)
	m = m + mirrorYZ()(m)
	
	hh = polygon([(0,-4), (-2,2), (-12,6), (5,8), (5,-4)]).extrude(hold_t).right(polyg_xseg*4-5).fillet(hold_fil2)

	c = circle(r=hold_t / 3 * 2).rotateY(deg(90)).forw(polyg_yseg*3.5)
	path = interpolate([
		(3*polyg_xseg, 		polyg_yseg*eey),
		(0*polyg_xseg, 		polyg_yseg*3.5),
		(-3*polyg_xseg, 	polyg_yseg*eey)
	])

	sph = sphere(hold_t / 3 * 2)
	det = pipe(c, path) + sph.translate(3*polyg_xseg, polyg_yseg*eey, 0) + sph.translate(-3*polyg_xseg, polyg_yseg*eey, 0)
	det = det.up(hold_t/2)

	m = m + det
	return m.rotateY(deg(90)).up(polyg_xseg*4).left(hold_t/2).forw(polyg_yseg*0.4 + 1.5)

def make_rebr(r, t, h, xblade, angle):
	rebr = cylinder(r=r, h=h) - cylinder(r=r-t, h=h)
	rebr = difference([
		rebr, 
		halfspace().rotateX(-deg(90)),
		halfspace().rotateY(deg(90)),
		halfspace().rotateY(-deg(90)-angle).right(xblade)
	])
	return rebr

######PARAMETRES######
r1 = 33 / 2
r2 = 38.5 / 2
r3 = r2*1.51
h = 7
base = 			make_body(con=(r1,r2,h), cil=(16,5), centrad=11.5, tooth=(54,0.8), forbolt=(27/2,1,deg(0)))
base_support = 	make_base_support(r1=r1, r2=r2, s=r3-r2, h=h, t=3/2)
holder = 		make_holder(t=3).forw(16.5)
rebr = 			make_rebr(r=r3, t=2, h=23, xblade=r2, angle=deg(30)).up(h)
######################
	
m = union([
	holder,
	base,
	base_support,
	rebr
])

m = unify(m)


MODEL = m