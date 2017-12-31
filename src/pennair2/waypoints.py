import math
import copy

class Waypoints:
	def __init__(self):
		self.queue = []

	# append tuple of coordinates to end of waypoints queue
	def WaypointsAppendTuple(self, tup):
		self.queue.append(tup)

	# appoint coordinates to end of waypoints queue
	def WaypointsAppendCoords(self, x, y, z):
		self.queue.append((x,y,z))

	# append list of tuples (assume constructed correctly) to queue
	def WaypointsAppendList(self, lst):
		for elem in lst:
			self.queue.append(elem)

	# return the number of elements in the waypoints queue
	def WaypointsCount(self):
		return (len(self.queue))

	# helper function to compute the distance between two points
	def DistHelper(tup1, tup2):
		powered = (tup1[0] - tup2[0])**2 + (tup1[1] - tup2[1])**2 + (tup1[2] - tup2[2])**2
		return math.sqrt(powered)

	# return the distance travelled along the given path
	def WaypointsDistance(self):
		prev = (0, 0, 0)
		total = 0
		for tup in self.queue:
			total += DistHelper(prev, tup)
			prev = tup 
		return total

	# pop first thing in the queue
	def WaypointsPopFirst(self):
		return self.queue.pop(0)

	# pop the item at index in the queue
	def WaypointsPopIndex(self, i):
		return self.queue.pop(i)

	# print out the entire queue
	def WaypointsPrint(self):
		print(self.queue)

	# return if a tuple exists in the queue
	def WaypointsContains(self, tup):
		return tup in self.queue

	# insert a tuple at the given index in the queue
	def WaypointsInsertIndex(self, tup, i):
		self.queue.insert(i, tup)

	# return a deep copy of the list of the queue
	def WaypointsDeepcopy(self):
		return copy.deepcopy(self.queue)

	# remove a tuple, do nothing if not in queue
	def WaypointsRemove(self, tup):
		if tup in self.queue:
			self.queue.remove(tup)

	# remove all coordinates in the queue that has x as the x coordinate
	def WaypointsRemoveX(self, x):
		self.queue = [tup for tup in self.queue if tup[0] != x]

	# remove all coordinates in the queue that has y as the y coordinate
	def WaypointsRemoveY(self, y):
		self.queue = [tup for tup in self.queue if tup[1] != y]

	# remove all coordinates in the queue that has z as the z coordinate
	def WaypointsRemoveZ(self, z):
		self.queue = [tup for tup in self.queue if tup[2] != z]

	# remove all items after the i'th index in the queue, starting from 0
	def WaypointsTruncate(self, i):
		deleting = []
		count = 0
		for x in self.queue:
			if count > i:
				deleting.append(x)
			count += 1
		for y in deleting:
			self.queue.remove(y)

	# clear everything
	def WaypointsClear(self):
		self.queue = []

	# remove all duplicate coordinates in the queue, retains 1st appearnace of each
	def WaypointsRemoveDups(self):
		tempSet = set(self.queue)
		deleting = []
		count = 0
		for tup in self.queue:
			if tup in tempSet:
				tempSet.discard(tup)
			else:
				deleting.append(count)
			count += 1
		deleting = deleting[::-1]
		for i in deleting:
			self.queue.pop(i)







