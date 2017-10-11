class Waypoints:
	def __init__(self):
		self.queue = []
		pass

	def addTupleWaypoint(self, tuple):
		self.queue.append(tuple)

	def addCoordWaypoint(self, x, y, z):
		tup = (x, y, z)
		self.queue.append(tup)

	def lengthWaypoints(self):
		return len(self.queue)

	def popWaypoint(self):
		return self.queue.pop(0)

	def printWaypoints(self):
		print(self.queue)

