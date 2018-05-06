import unittest
import pennair2.navigation.waypoint


class TestWaypoint(unittest.TestCase):

    def test_contructor(self):
        points = pennair2.navigation.waypoint.Waypoints("euclidean")
        self.assertEqual(points.waypoints, [])
        self.assertEqual(points.index, 0)

    def test_add(self):
        points = pennair2.navigation.waypoint.Waypoints("euclidean")
        points.add([0, 0, 0])
        self.assertEqual(points.waypoints, [[0, 0, 0]])
        points.add([1, 1, 1])
        self.assertEqual(points.waypoints, [[0, 0, 0], [1, 1, 1]])
        points.add([2, 2, 2])
        self.assertEqual(points.waypoints, [[0, 0, 0], [1, 1, 1], [2, 2, 2]])
        points.add([3, 3, 3])
        self.assertEqual(points.waypoints, [[0, 0, 0], [1, 1, 1], [2, 2, 2],
                                            [3, 3, 3]])
        points.add([4, 4, 4], 0)
        self.assertEqual(points.waypoints, [[4, 4, 4], [0, 0, 0], [1, 1, 1],
                                            [2, 2, 2], [3, 3, 3]])

    def test_delete(self):
        points = pennair2.navigation.waypoint.Waypoints("euclidean")
        points.add([0, 0, 0])
        points.add([1, 1, 1])
        points.add([2, 2, 2])
        points.add([3, 3, 3])
        points.delete(1)
        self.assertEqual(points.waypoints, [[0, 0, 0], [2, 2, 2], [3, 3, 3]])
        points.delete(0)
        self.assertEqual(points.waypoints, [[2, 2, 2], [3, 3, 3]])
        points.delete(1)
        self.assertEqual(points.waypoints, [[2, 2, 2]])

    def test_next(self):
        points = pennair2.navigation.waypoint.Waypoints("euclidean")
        points.add([0, 0, 0])
        points.add([1, 1, 1])
        points.add([2, 2, 2])
        self.assertEqual(points.next(), [0, 0, 0])
        self.assertEqual(points.index, 1)
        self.assertEqual(points.next(), [1, 1, 1])
        self.assertEqual(points.index, 2)
        self.assertEqual(points.next(), [2, 2, 2])
        self.assertEqual(points.index, 0)

    def test_goto(self):
        points = pennair2.navigation.waypoint.Waypoints("euclidean")
        points.add([0, 0, 0])
        points.add([1, 1, 1])
        points.add([2, 2, 2])
        points.add([3, 3, 3])
        self.assertEqual(points.goto(3), [3, 3, 3])
        self.assertEqual(points.index, 3)
        self.assertEqual(points.goto(1), [1, 1, 1])
        self.assertEqual(points.index, 1)

    def test_length(self):
        points = pennair2.navigation.waypoint.Waypoints("euclidean")
        self.assertEqual(points.length(), 0)
        points.add([0, 0, 0])
        self.assertEqual(points.length(), 1)
        points.add([1, 1, 1])
        points.add([2, 2, 2])
        points.add([3, 3, 3])
        self.assertEqual(points.length(), 4)
