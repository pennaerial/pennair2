#Simple class to manage waypoints
class Waypoints():

    #Define two fields, empty list of waypoints and index
    def __init__(self, frame : str):
        self.waypoints = []
        self.index = 0

    #add a waypoint by default to the end or at a specified index
    def add(self, waypoint : list, index = 'unspecified'):
        if index is 'unspecified':
            self.waypoints.append(waypoint)
        else:
            self.waypoints.insert(index, waypoint)

    #delete waypoint at the index
    def delete(self, index : int):
        del self.waypoints[index]

    #return waypoint at current index and increment index with wrapping
    def next(self):
        output = self.waypoints[self.index]
        self.index += 1
        if self.index is len(self.waypoints):
            self.index = 0
        return output

    #change index and return waypoint at new index
    def goto(self, index : int):
        self.index = index
        return self.waypoints[index]

    #return length
    def length(self):
        return len(self.waypoints)

#TESTING
points = Waypoints("euclidean")
points.add([0, 0, 0])
print(points.waypoints)
points.add([1, 1, 1])
print(points.waypoints)
points.add([2, 2, 2])
print(points.waypoints)
points.add([3, 3, 3])
print(points.waypoints)
print(points.next())
print("Length: ", points.length(), " Index: ", points.index)
print(points.goto(3))
print(points.next())
print(points.next())
points.delete(0)
print(points.waypoints)
