'''
@Author: Rene Jacques
March 17, 2019
'''

class Node:
	'''Node used as a storage object to keep track of specific data, node location in tree (ID), and node parent'''
	def __init__(self):
		self.data = None
		self.data_number = None
		self.parent = None
		self.id = None
		self.cost = None

	def setCost(self,c):
		'''Set cost to reach for this node'''
		self.cost = c 

	def getCost(self):
		'''Return cost to reach'''
		return self.cost

	def setData(self,d):
		'''Set data for this node'''
		self.data = d 

	def getData(self):
		'''Return data'''
		return self.data

	def setID(self,i):
		'''Set node ID'''
		self.id = i

	def getID(self):
		'''Return node ID'''
		return self.id

	def setParent(self,p):
		'''Set node parent'''
		self.parent = p

	def getParent(self):
		'''Return node parent'''
		return self.parent

class H_Node(Node):
	'''Node with Heuristic (Extends Node)'''
	def __init__(self,data=0):
		Node.__init__(self)
		self.coords = [None,None]
		self.g = 0 # cost to come
		self.h = 0 # distance to goal
		self.f = self.g+self.h # g+h
		self.data = data

		self.child = None
		self.pathParent = None
		self.pathChild = None

	def setCoords(self,x,y):
		'''Set (x,y) coordinate pair for this node'''
		self.coords = [x,y]

	def getCoords(self):
		'''Get (x,y) coordinate pair for this node'''
		return self.coords

	def setCost(self,g,h):
		'''Set g and h cost for this node'''
		self.g = g
		self.h = h
		self.f = g+h
		self.setData(self.f)

	def getCost(self):
		'''Get g, h, and f costs for this node'''
		return self.g,self.h,self.f

	def setChild(self,n):
		'''Set child for this node'''
		self.child = n

	def getChild(self):
		'''Get child for this node'''
		return self.child

	def setPathParent(self,n):
		'''Set the path parent for this node'''
		self.pathParent = n

	def getPathParent(self):
		'''Get the path parent for this node'''
		return self.pathParent

	def setPathChild(self,n):
		'''Set the path child for this node'''
		self.pathChild = n

	def getPathChild(self):
		'''Get the path child for this node'''
		return self.pathChild

class D_Node(H_Node):
	def __init__(self,data=0):
		H_Node.__init__(self,data)

		self.rpm = [None,None]
		self.angle = 0

	def setRPM(self,rpm):
		self.rpm = rpm

	def getRPM(self):
		return self.rpm

	def setAngle(self,angle):
		self.angle = angle

	def getAngle(self):
		return self.angle