'''
@Author: Rene Jacques
March 18, 2019
'''

import cv2, math
import numpy as np

# define colors
red = (0,0,255)
green = (0,255,0)
blue = (255,0,0)
black = (0,0,0)
white = (255,255,255)

def line_eq(p1,p2):
	'''Equation for a line derived from two points'''
	slope = 1.0*(p2[0]-p1[0])/(p2[1]-p1[1])
	intercept = p2[0]-slope*p2[1]

	return slope,intercept

def ellipse_eq(center,width,height,x,y):
	'''Checks if x,y are within an ellipse described by center, width, height'''
	if (((1.0*center[0]-x)**2)/(width**2))+(((1.0*center[1]-y)**2)/(height**2)) <= 1:
		return True	
	return False

def circle_eq(center,x,y,radius):
	'''Checks if x,y are within a circle described by center, radius'''
	if (center[0]-x)**2+(center[1]-y)**2 <= radius**2:
		return True
	return False

def mink_sum(A,B):
	'''Enlarges shape A with shape B using Minkowski Sum'''
	output = []
	for a in A:
		for b in B:
			# sum Ax, Bx and sum Ay, By to get the new shape
			output.append([a[0]+b[0],a[1]+b[1]])
	return output

class Map_Builder:
	'''Builds the map of the obstacles as well as the grid space describing the obstacles'''
	def __init__(self,size,px_unit=1):
		self.images = {}
		self.size = [int(size[0]*px_unit),int(size[1]*px_unit)]
		print(self.size)
		self.shapes = {'rectangles':[],'circles':[],'ellipses':[]}

		self.number = 0

		self.px_unit = px_unit

	def add_image(self,name):
		img = np.zeros((self.size[1],self.size[0]),np.uint8)
		img[np.where(img==0)] = 255

		img = cv2.cvtColor(img,cv2.COLOR_GRAY2BGR)

		self.images[name] = img

	def add_shape(self,shape,center=None,radius=0,width=0,height=0,points=None,test=None,logic=None):
		if shape == 'rectangle':
			self.shapes['rectangles'].append([(center[0]*self.px_unit,center[1]*self.px_unit),width*self.px_unit,height*self.px_unit])
		elif shape == 'circle':
			self.shapes['circles'].append([(center[0]*self.px_unit,center[1]*self.px_unit),radius*self.px_unit])
		elif shape == 'ellipse':
			self.shapes['ellipses'].append([(center[0]*self.px_unit,center[1]*self.px_unit),width*self.px_unit,height*self.px_unit])

	def create_map(self,res,clearance,radius,draw=False):
		'''Creates the maps'''
		res = int(res)
		if res == 0:
			res = 1

		dist = (clearance+radius)*self.px_unit

		# # # define robot shape
		# rob_x = np.linspace(-dist,dist,num=1+2*dist,dtype=np.int32)
		# rob_y = np.linspace(-dist,dist,num=1+2*dist,dtype=np.int32)

		# rob_circle = []

		# # # create list of points in robot circle
		# for x in rob_x:
		# 	for y in rob_y:
		# 		if circle_eq((0,0),x,y,dist):
		# 			rob_circle.append([x,y])

		# rob_circle = np.array([rob_circle])
		# rob_circle = rob_circle[0]

		points = []
		for row in range(self.size[1]):
			for col in range(self.size[0]):
				for rect in self.shapes['rectangles']:
					center,w,h = rect 
					if col >= center[0] and col <= center[0]+w and row >= center[1] and row <= center[1]+h:
						self.images['orig'][row,col] = 0

					if col >= center[0] and col <= center[0]+w and ((row >= center[1]-dist and row <= center[1]) or (row >= center[1]+h and row <= center[1]+h+dist)):
						self.images['orig'][row,col] = 0

					if ((col >= center[0]-dist and col <= center[0]) or (col >= center[0]+w and col <= center[0]+w+dist)) and row >= center[1] and row <= center[1]+h:
						self.images['orig'][row,col] = 0

					if circle_eq(center,col,row,dist) or circle_eq((center[0]+w,center[1]),col,row,dist) or circle_eq((center[0]+w,center[1]+h),col,row,dist) or circle_eq((center[0],center[1]+h),col,row,dist):
						self.images['orig'][row,col] = 0

				for circ in self.shapes['circles']:
					center,r = circ
					if circle_eq(center,col,row,r+dist):
						self.images['orig'][row,col] = 0

				if col >= 0 and col <= dist:
					self.images['orig'][row,col] = 0

				if col >= self.size[0]-dist and col <= self.size[0]:
			 		self.images['orig'][row,col] = 0

			 	if row >= 0 and row <= dist:
			 		self.images['orig'][row,col] = 0

			 	if row >= self.size[1]-dist and row <= self.size[1]:
			 		self.images['orig'][row,col] = 0

		# print('mink sum')
		# ms = mink_sum(points,rob_circle)

		# print('color in')
		# for p in ms:
		# 	if p[1] >= 0 and p[0] >= 0:
		# 		self.images['mink'][p[1],p[0]] = 0

		cv2.imshow('orig',cv2.resize(self.images['orig'],(0,0),fx=0.75,fy=0.75))
		# cv2.imshow('mink',cv2.resize(self.images['mink'],(0,0),fx=0.75,fy=0.75))
		# cv2.imwrite('obstacle_map_mink_border.png',self.images['orig'])
		cv2.waitKey(0)

def test():
	print('start')

	m = Map_Builder((11.1,10.1),px_unit=100)
	m.add_image('orig')
	m.add_image('grid')
	m.add_image('mink')
	m.add_image('mink_grid')
	m.add_image('mink_map')

	m.add_shape('rectangle',(11.1-(1.92+0.86),0),width=0.86,height=1.83)
	m.add_shape('rectangle',(11.1-(0.84+0.43),0),width=0.43,height=0.91)
	m.add_shape('rectangle',(11.1-(3.66),3.13),width=3.66,height=0.76)
	m.add_shape('rectangle',(11.1-(0.58),3.13+0.76+0.555),width=0.58,height=1.17)
	m.add_shape('rectangle',(11.1-(0.91),3.13+0.76+0.555+1.17),width=0.91,height=0.86)
	m.add_shape('rectangle',(11.1-(0.58),3.13+0.76+0.555+1.17+0.86+0.6725),width=0.58,height=1.17)
	m.add_shape('rectangle',(11.1-(4.25),10.1-0.35),width=4.25,height=0.35)
	m.add_shape('rectangle',(11.1-(1.83),10.1-0.35-0.76),width=1.83,height=0.76)
	m.add_shape('rectangle',(11.1-(1.83+0.31+1.17),10.1-(0.35+0.58)),width=1.17,height=0.58)
	m.add_shape('rectangle',(11.1-(1.83+0.31+1.17+0.31+2.74),10.1-(0.35+1.52)),width=2.74,height=1.52)
	m.add_shape('rectangle',((4.38+0.91+1.83+0.725),10.1-(0.35+1.52+0.8+1.17)),width=1.52,height=1.17)
	m.add_shape('rectangle',((4.38+0.91),10.1-(0.35+1.52+0.78+0.76)),width=1.83,height=0.76)
	m.add_shape('rectangle',((4.38),10.1-(0.35+1.52+1.28+1.83)),width=0.91,height=1.83)

	m.add_shape('rectangle',((0.7+1.599/2),(1)),width=3.1968-1.599,height=1.599)
	m.add_shape('circle',(0.7+1.599/2,1+1.599/2),radius=1.599/2)
	m.add_shape('circle',(0.7+3.1968-1.599/2,1+1.599/2),radius=1.599/2)

	m.add_shape('circle',(3.9,0.45),radius=0.81/2)
	m.add_shape('circle',(4.38,2.74),radius=0.81/2)
	m.add_shape('circle',(3.9,10.1-0.45),radius=0.81/2)
	m.add_shape('circle',(4.38,10.1-(0.87+1.52+0.35)),radius=0.81/2)
	# print(m.shapes)
	m.create_map(0,0.05,0.177,draw=True)
	print('finished')

if __name__ == '__main__':
	test()