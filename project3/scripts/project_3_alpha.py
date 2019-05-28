#!/usr/bin/env python

import cv2, copy
import numpy as np
import math
from A_star_differential_charlie import A_star
from path_publisher import PathPublisher
import rospy
print(cv2.__version__)

red = (0,0,255)
green = (0,255,0)
blue = (255,0,0)

step = 2
L = 0.27 #0.27
r = 0.038

# cv2.imshow('orig',cv2.resize(img,(0,0),fx=0.5,fy=0.5))
# cv2.imshow('map',cv2.resize(obst,(0,0),fx=0.5,fy=0.5))
# cv2.waitKey(0)

def angle(vl,vr):
	return (r/L)*(vr-vl)*step

def dist(xy,vl,vr,angle):
	if xy == 'x':
		return (r/2)*(vl+vr)*math.cos(angle)*step
	elif xy == 'y':
		return (r/2)*(vl+vr)*math.sin(angle)*step

class App:
	def __init__(self,res,start,goal,rpm):
		self.res = res
		self.grid = {}
		self.start = start
		self.goal = goal
		self.rpm = rpm

		start_x = int(float(rospy.get_param("coords/start_x")+5.55)*100//res)
		start_y = int(abs(float(rospy.get_param("coords/start_y")-5.05))*100//res)
		goal_x = int(float(rospy.get_param("coords/goal_x")+5.55)*100//res)
		goal_y = int(abs(float(rospy.get_param("coords/goal_y")-5.05))*100//res)

		# start_x = int((float('-5.05')+5.55)*100//res)
		# start_y = int(abs((float('-4')-5.05))*100//res)
		# goal_x = int((float('-2')+5.55)*100//res)
		# goal_y = int(abs((float('-1')-5.05))*100//res)
		
		self.start = [start_x,start_y]
		self.goal = [goal_x,goal_y]

		img = cv2.imread('/home/rcj/catkin_ws/src/project3/scripts/obstacle_map_mink_border.png')
		# print(img)

		self.run(img)

	def make_grid(self,img):
		for i in range(img.shape[1]):
			for j in range(img.shape[0]):
				x = int(1.0*i/self.res)
				y = int(1.0*j/self.res)
				if any(img[j,i]) == 0:
					# obst[j,i] = 0
					self.grid[x,y] = 1
				else:
					self.grid[x,y] = 0

	def run(self,img):
		print(self.start,self.goal)
		self.make_grid(img)
		obst,grid_image = self.draw_output(img,draw=True)

		a_star = A_star(self.grid,grid_image,obst,self.res,px_unit=100)
		path,final_image = a_star.search(self.start,self.goal,self.rpm)
		print(path)

		cv2.imshow('A*',cv2.resize(final_image,(0,0),fx=0.5,fy=0.5))

		p = PathPublisher(radius=0.076/2,l=0.27)
		p.run(path)

		cv2.waitKey(0)

	def draw_output(self,img,draw=False):
		obst = np.ones(img.shape,np.uint8)*255

		# space = np.ones((25,50))*255

		# options = [[0,self.rpm[0]],[self.rpm[0],0],[0,self.rpm[1]],[self.rpm[1],0],[self.rpm[0],self.rpm[1]],[self.rpm[1],self.rpm[0]],[self.rpm[0],self.rpm[0]],[self.rpm[1],self.rpm[1]]]

		# for v in options:
		# 	v[0] = v[0]*np.pi/30
		# 	v[1] = v[1]*np.pi/30

		# 	a = angle(v[0],v[1])
		# 	vx = int(1.0*(dist('x',v[0],v[1],a)*100)/self.res)
		# 	vy = int(1.0*(dist('y',v[0],v[1],a)*100)/self.res)

		# 	print(vx,vy)

		# 	space[(space.shape[0]/2)+vy,10+vx] = 0
		# 	# cv2.(space,(10+vx,space.shape[0]/2+vy),1,0,-1)

		# cv2.imshow('8 space',cv2.resize(space,(0,0),fx=10,fy=10))
		# cv2.waitKey(0)

		for key in self.grid:
			if self.grid[key] == 1:
				x = key[0]*self.res
				y = key[1]*self.res
				cv2.rectangle(obst,(x,y),(x+res,y+res),0,-1)

		cv2.rectangle(obst,(self.start[0]*res,self.start[1]*res),(res*self.start[0]+res,res*self.start[1]+res),green,-1)
		cv2.rectangle(obst,(self.goal[0]*res,self.goal[1]*res),(res*self.goal[0]+res,res*self.goal[1]+res),red,-1)

		grid_image = copy.copy(obst)

		for i in range(6):
			cv2.line(grid_image,(obst.shape[1]/2+i*100,0),(obst.shape[1]/2+i*100,obst.shape[0]),50,1)
			cv2.line(grid_image,(obst.shape[1]/2-i*100,0),(obst.shape[1]/2-i*100,obst.shape[0]),50,1)

		for j in range(12):
			cv2.line(grid_image,(0,j*100),(obst.shape[1],j*100),50,1)
			# cv2.line(grid_image,(0,obst.shape[1]/2+j*100),(obst.shape[1],obst.shape[1]/2+j*100),50,1)
			# cv2.line(grid_image,(0,obst.shape[1]/2-j*100),(obst.shape[1],obst.shape[1]/2-j*100),50,1)
		# if self.res >= 3:
		# 	for i in range(obst.shape[1]//res+1):
		# 		cv2.line(grid_imageobst.shape[0]),50,1),(i*res,0),(i*res,obst.shape[0]),50,1)

		# 	for j in range(obst.shape[0]//res+1):
		# 		cv2.line(grid_image,(0,j*res),(obst.shape[1],j*res),50,1)

		if draw:
			cv2.imshow('A*',cv2.resize(cv2.bitwise_and(obst,grid_image),(0,0),fx=0.5,fy=0.5))
			cv2.imwrite('obstacle_map_mink_border_5.png',obst)
			cv2.waitKey(0)

		return obst,grid_image

if __name__ == '__main__':
	# rospy.loginfo('Start')
	print('correct')
	res = 10
	start = (50//res,300//res) # -5.05, 2.05 //50 300
	goal = (200//res,700//res) # -3.55, -1.95 //200 700
	a = App(res,start,goal,[50,100])
	# rospy.loginfo('Finished')