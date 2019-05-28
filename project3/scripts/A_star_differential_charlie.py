'''
@Author: Rene Jacques
March 21, 2019
'''

from node import D_Node
from priority_queue import NaivePriorityQueue
import math, cv2
import numpy as np

# define grid space values
WALL = 1
SPACE = 0
OPEN = 2
CLOSED = 3

# define colors
yellow = (0,255,255)
cyan = (255,255,0)
white = (255,255,255)
black = (0,0,0)
gray = (50,50,50)
blue = (255,0,0)
green = (0,255,0)
red = (0,0,255)

step = 1
L = 0.27 #0.27
r = 0.038

def distance(one,two):
	'''Calculate straight line distance between two points'''
	x1,y1 = one.getCoords()
	x2,y2 = two.getCoords()
	return math.sqrt((x1-x2)**2+(y1-y2)**2)

def arc_length(vl,vr):
	if vr-vl != 0:
		radius = (L/2)*(vl+vr)/(vr-vl)
		angle = angle(vl,vr )
	else:
		return 0
	return angle*radius

def angle(vl,vr):
	return (r/L)*(vr-vl)*step

# def dist(xy,vl,vr,angle):
# 	if xy == 'x':
# 		return (r/2)*(vl+vr)*math.cos(angle)*step
# 	elif xy == 'y':
# 		return (r/2)*(vl+vr)*math.sin(angle)*step

def turn_radius(vl,vr):
	if vr-vl != 0:
		return (L/2)*(vl+vr)/(vr-vl)
	else:
		return False

def dist(xy,vl,vr,angle,da):
	R = turn_radius(vl,vr)
	if xy == 'x':
		if R != False:
			dx = 2*R*np.sin(da/2)*np.cos((angle+da)/2)
			# print('dx',dx)
			return dx
		else:
			dx = vl*np.cos((angle))*r
			# print('dx',dx)
			return dx
	elif xy == 'y':
		if R != False:
			dy = 2*R*np.sin(da/2)*np.sin((angle+da)/2)
			# print('dy',dy)
			return dy
		else:
			dy = vl*np.sin((angle))*r
			# print('dy',dy)
			return dy

class A_star:
	'''A* Search Algorithm'''
	def __init__(self,grid,lines,img,res,px_unit=1):
		self.grid = grid
		self.lines = lines
		self.img = img
		self.res = res
		self.px_unit = px_unit

		self.open_set = NaivePriorityQueue() 
		self.closed_set = set()

	def img_test(self):
		'''Test if image is correct'''
		cv2.rectangle(self.img,(self.res*10,self.res*10),(self.res*10+50,self.res*10+50),yellow,-1)
		cv2.imshow('A*',cv2.resize(self.img,(0,0),fx=0.5,fy=0.5))

		cv2.waitKey(0)

	def search(self,start,goal,rpm):
		'''Search for goal coordinates given start coordinates'''
		self.rpm = rpm
		x,y = start
		# color in start square
		cv2.rectangle(self.img,(self.res*x,self.res*y),((self.res*x+self.res),(self.res*y+self.res)),blue,-1)

		x,y = goal
		# color in goal square
		cv2.rectangle(self.img,(self.res*x,self.res*y),((self.res*x+self.res),(self.res*y+self.res)),red,-1)

		# create start node
		s = D_Node()
		s.setCoords(start[0],start[1])

		# create goal node
		g = D_Node()
		g.setCoords(goal[0],goal[1])

		# add start node to open_set
		self.open_set.add(s)

		# initialize variable to check if the size of the open_set stays 1 for 5 loops
		open_count = 0

		# search until the open_set is empty or a break is encountered
		while not self.open_set.isEmpty() and open_count != 5:
			if(len(self.open_set.getQueue()) == 1):
				open_count += 1
			else:
				open_count = 0

			# get the next node out of the open set
			next_node = self.open_set.pop()

			# get the next nodes neighbors
			neighbors = self.get_neighbors(next_node)

			for neighbor in neighbors:
				# check if the neighbor is at the goal
				# if neighbor.getCoords()[0]==goal[0] and neighbor.getCoords()[1]==goal[1]:
				x,y = neighbor.getCoords()
				if math.sqrt((x-goal[0])**2+(y-goal[1])**2)/self.res <= 10/self.res:
					print('Goal Found At:'+str(neighbor.getCoords()))

					# show the path from goal to start
					# self.path = np.ones((500,500),np.uint8)*255
					path = []
					path = self.build_path(neighbor,path)
					print('start',start,'goal',goal)

					# draw the gridlines on the final output image
					# cv2.imshow('Path',self.path)
					grid_img = cv2.bitwise_and(self.img,self.lines)
					cv2.imshow('A*',cv2.resize(grid_img,(0,0),fx=0.5,fy=0.5))

					cv2.waitKey(0)
					return path,grid_img
				else:
					# calculate the g and h costs for the neighbor
					# print(next_node.getAngle(),neighbor.getAngle(),next_node.getAngle() != neighbor.getAngle())
					if next_node.getAngle() != neighbor.getAngle():
						g_cost = next_node.getCost()[0]+distance(next_node,neighbor) # neighbor g cost = parent node's g cost + distance between neighbor and parent
					else:
						g_cost = next_node.getCost()[0]+arc_length(neighbor.getRPM()[0],neighbor.getRPM()[1])
					g_cost = next_node.getCost()[0]+distance(next_node,neighbor)
					h_cost = distance(neighbor,g) # neighbor h cost = distance between neighbor and goal
					neighbor.setCost(g_cost,h_cost)

					# initialize a flag to indicate duplicates in the open and closed sets
					duplicate_flag = False

					# check if the neighbor is in the open set or the closed set
					for item in self.open_set.getQueue():
						# check if the neighbor is in the closed set
						if str(neighbor.getCoords()) in self.closed_set:
							duplicate_flag = True
							break

						if not duplicate_flag:
							# check if the neighbor is in the open set
							if item.getCoords() == neighbor.getCoords():
								# if the neighbor's g cost is less than a node's g cost 
								if neighbor.getCost()[0] < item.getCost()[0]:
									self.open_set.remove(item) # remove the node from the open set
									self.open_set.add(neighbor) # add the neighbor to the open set
								duplicate_flag = True
								break

					# if the neighbor is not in the open set and not in the closed set
					if not duplicate_flag:
						# add neighbor to the open set
						self.open_set.add(neighbor)
						x,y = neighbor.getCoords()

						# if the neighbor is not the start node and not the goal node 
						if (x != start[0] or y != start[1]) and (x != goal[0] or y != goal[1]):
							# color in the neighbor's grid square as yellow
							cv2.rectangle(self.img,(self.res*x,self.res*y),((self.res*x+self.res),(self.res*y+self.res)),yellow,-1)

			# add the current node to the closed set
			closed = str(next_node.getCoords())
			self.closed_set.add(closed)
			x,y = next_node.getCoords()

			# if the current node is not the start node and not the goal node
			if (x != start[0] or y != start[1]) and (x != goal[0] or y != goal[1]):
				# color in the current node's grid square as cyan
				cv2.rectangle(self.img,(self.res*x,self.res*y),((self.res*x+self.res),(self.res*y+self.res)),cyan,-1)

			# draw the grid lines and display the animation
			grid_img = cv2.bitwise_and(self.img,self.lines)
			cv2.imshow('A*',cv2.resize(grid_img,(0,0),fx=0.5,fy=0.5))
			# cv2.waitKey(0)

			if cv2.waitKey(1) & 0xFF == ord('q'):
				break

		# if the open set is empty then there is no path possible
		if self.open_set.isEmpty() or open_count ==5:
			print('No Path Possible')
		cv2.waitKey(0)

	def get_neighbors(self,n):
		'''Get all of the neighbors for the input node'''
		neighbors = []
		options = [[0,self.rpm[0]],[self.rpm[0],0],[0,self.rpm[1]],[self.rpm[1],0],[self.rpm[0],self.rpm[1]],[self.rpm[1],self.rpm[0]],[self.rpm[0],self.rpm[0]],[self.rpm[1],self.rpm[1]]]

		x,y = n.getCoords()

		current_angle = n.getAngle()

		# get each neigbor to the top left, top middle, top right, right middle, bottom right, bottom middle, bottom left, left middle
		for v in options:
			v[0] = v[0]*np.pi/30
			v[1] = v[1]*np.pi/30
			# print(v)
			a = angle(v[0],v[1])
			vx = int(1.0*(dist('x',v[0],v[1],current_angle,a)*self.px_unit)/self.res)
			vy = int(1.0*(dist('y',v[0],v[1],current_angle,a)*self.px_unit)/self.res)
			# print(int(v[0]),int(v[1]),vx,vy,current_angle)
			# make sure that the coordinates are not outside of the map
			if x+vx < 0 or y+vy < 0 or x+vx > (self.img.shape[1])//self.res-1 or y+vy > (self.img.shape[0])//self.res-1:
				pass

			# if the coordinates are not in a wall then create a new node and add to the neighbor list 
			elif self.grid[x+vx,y+vy] != WALL:
				neighbor = D_Node()
				neighbor.setCoords(x+vx,y+vy)
				neighbor.setPathParent(n)
				neighbor.setAngle(current_angle+a)
				neighbor.setRPM(v)
				neighbors.append(neighbor)

		return neighbors

	def build_path(self,n,path):
		'''Recursive function to build the path by searching through the parents of each node from start to goal'''
		x,y = n.getCoords()
		rpm = n.getRPM()
		# angle = n.getAngle()

		print(x,y,rpm)

		# color in the path squares
		cv2.rectangle(self.img,(self.res*x,self.res*y),((self.res*x+self.res),(self.res*y+self.res)),red,-1)
		# print(n.getRPM(),np.rad2deg(n.getAngle()),n.getCoords())
		# check if the next parent exists (if it does not then the end of the path has been reached)
		path.insert(0,rpm)
		if n.getPathParent() != None:
			# xp,yp = n.getPathParent().getCoords()
			# rpm_p = n.getPathParent().getRPM()
			# angle_p = n.getPathParent().getAngle()
			# xm = (x+xp)/2
			# ym = (y+yp)/2
			# d = int(distance(n,n.getPathParent()))
			# print(d)
			# cv2.ellipse(self.path,(xm*self.res,ym*self.res),(d,20),0,0,360,black,1)
			self.build_path(n.getPathParent(),path)
		return path