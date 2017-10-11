class Waypoints:
	def __init__(self):
		self.queue = []
		pass

	def addTupleWaypoint(self, tuple):
		self.queue.append(tuple)

	def addCoordWaypoint(self, x, y, z):
		tup = (x, y, z)
		self.queue.append(tup)

