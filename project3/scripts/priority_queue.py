'''
@Author: Rene Jacques
March 24, 2019
'''

from node import H_Node

class NaivePriorityQueue:
	'''Naive Priority Queue implementation. Searches through its queue until it finds a higher cost then the number that is being added and then 
	inserts that number into the correct index.'''
	def __init__(self):
		self.queue = [] 

	def add(self,item):
		'''Add item to queue by inserting behind the next lowest cost item in the queue'''
		if self.isEmpty():
			# check if queue is empty and if so add the item to the queue
			self.queue.append(item)
		else:
			# if queue already has items
			for n in self.queue:
				# iterate through every item in the queue
				if item.getData() < n.getData():
					# if the items priority is less than the priority of the node insert the item in front of the node
					 self.queue.insert(self.queue.index(n),item)
					 return
				elif item.getData() > n.getData():
					# if the items priority is greater than the nodes priority
					if self.queue.index(n) == len(self.queue)-1:
						# check if the node is as the end of the queue and if so append the item to the queue
						self.queue.append(item)
				elif item.getData() == n.getData():
					# if the items priority is equal to the nodes priority
					if self.queue.index(n) < len(self.queue)-1:
						# if the node is not at the end of the queue
						if self.queue[self.queue.index(n)+1].getData() > item.getData():
							# if the node behind the current node has a larger priority than the item then insert item in front of the larger priority node
							self.queue.insert(self.queue.index(n)+1,item)
							return

	def pop(self):
		'''Remove the first item in the queue, which will be the item with the highest priority'''
		return self.queue.pop(0)

	def isEmpty(self):
		'''Check if queue is empty'''
		return len(self.queue) == 0

	def getQueue(self):
		'''Get queue'''
		return self.queue

	def print_queue(self):
		'''Print out the queue for testing purposes'''
		for item in self.queue:
			print(item.getData())

	def remove(self,n):
		'''Remove an item from the queue'''
		for item in self.queue:
			if n.getCoords()[0] == item.getCoords()[0] and n.getCoords()[1] == item.getCoords()[1]:
				self.queue.pop(self.queue.index(item))
				break

def test():
	p = NaivePriorityQueue()
	p.add(H_Node(1))
	p.add(H_Node(3))
	p.add(H_Node(5))
	p.add(H_Node(7))
	p.add(H_Node(8))
	p.add(H_Node(2))
	p.add(H_Node(4))
	p.add(H_Node(3))
	p.add(H_Node(9))
	p.add(H_Node(9))

	p.print_queue()

if __name__ == '__main__':
	test()