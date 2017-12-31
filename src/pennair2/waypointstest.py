import unittest
from waypoints import Waypoints

class WaypointsTest(unittest.TestCase):

	def setUp(self):
		self.w = Waypoints()

	def testEmpty(self):
		self.assertEqual(self.w.WaypointsCount(), 0)
		self.assertEqual(self.w.WaypointsDeepcopy(), [])

	def testCount(self):
		self.w.WaypointsAppendTuple((0, 1, 2))
		self.w.WaypointsAppendCoords(4, 5, 6)
		self.assertEqual(self.w.WaypointsCount(), 2)
		self.assertTrue(self.w.WaypointsContains((0, 1, 2)))
		self.assertTrue(self.w.WaypointsContains((4, 5, 6)))
		self.assertFalse(self.w.WaypointsContains((0, 0, 0)))

	def testList(self):
		lst = [(0, 0, 0), (1, 1, 1), (2, 2, 2)]
		self.w.WaypointsAppendList(lst)
		self.assertEqual(self.w.WaypointsCount(), 3)
		self.assertTrue(self.w.WaypointsContains((2, 2, 2)))
		self.assertTrue(self.w.WaypointsContains((1, 1, 1)))
		self.assertTrue(self.w.WaypointsContains((0, 0, 0)))

	def testPop(self):
		lst = [(0, 0, 0), (1, 1, 1), (2, 2, 2)]
		self.w.WaypointsAppendTuple((5, 6, 7))
		self.w.WaypointsAppendList(lst)
		self.assertEqual(self.w.WaypointsDeepcopy(), [(5, 6, 7), (0, 0, 0), (1, 1, 1), (2, 2, 2)])
		self.assertEqual(self.w.WaypointsPopFirst(), (5, 6, 7))
		self.assertEqual(self.w.WaypointsCount(), 3)
		self.assertEqual(self.w.WaypointsPopFirst(), (0, 0, 0))
		self.assertEqual(self.w.WaypointsCount(), 2)
		self.assertEqual(self.w.WaypointsPopFirst(), (1, 1, 1))
		self.assertEqual(self.w.WaypointsCount(), 1)
		self.assertEqual(self.w.WaypointsPopFirst(), (2, 2, 2))

	def testPopIndex(self):
		lst = [(0, 0, 0), (1, 1, 1), (2, 2, 2)]
		self.w.WaypointsAppendList(lst)
		self.w.WaypointsAppendTuple((5, 6, 7))
		self.assertEqual(self.w.WaypointsDeepcopy(), [(0, 0, 0), (1, 1, 1), (2, 2, 2), (5, 6, 7)])
		self.assertEqual(self.w.WaypointsCount(), 4)
		self.assertEqual(self.w.WaypointsPopIndex(1), (1, 1, 1))
		self.assertEqual(self.w.WaypointsCount(), 3)
		self.assertEqual(self.w.WaypointsPopIndex(2), (5, 6, 7))
		self.assertEqual(self.w.WaypointsCount(), 2)

	def testRemove(self):
		lst = [(0, 0, 0), (1, 1, 1), (2, 2, 2)]
		self.w.WaypointsAppendList(lst)
		self.w.WaypointsAppendTuple((5, 6, 7))
		self.assertEqual(self.w.WaypointsCount(), 4)
		self.w.WaypointsRemove((3, 3, 3))
		self.assertEqual(self.w.WaypointsCount(), 4)
		self.w.WaypointsRemove((2, 2, 2))
		self.assertEqual(self.w.WaypointsDeepcopy(), [(0, 0, 0), (1, 1, 1), (5, 6, 7)])


	def testRemoveX(self):
		lst = [(0, 1, 2), (1, 1, 1), (0, 2, 3), (4, 5, 6)]
		self.w.WaypointsAppendList(lst)
		self.assertEqual(self.w.WaypointsCount(), 4)
		self.w.WaypointsRemoveX(0)
		self.assertEqual(self.w.WaypointsDeepcopy(), [(1, 1, 1), (4, 5, 6)])

	def testRemoveY(self):
		lst = [(0, 1, 2), (1, 1, 1), (0, 2, 3), (4, 5, 6)]
		self.w.WaypointsAppendList(lst)
		self.assertEqual(self.w.WaypointsCount(), 4)
		self.w.WaypointsRemoveY(1)
		self.assertEqual(self.w.WaypointsDeepcopy(), [(0, 2, 3), (4, 5, 6)])

	def testRemoveZ(self):
		lst = [(0, 1, 2), (1, 1, 1), (0, 2, 3), (4, 5, 6)]
		self.w.WaypointsAppendList(lst)
		self.assertEqual(self.w.WaypointsCount(), 4)
		self.w.WaypointsRemoveZ(1)
		self.assertEqual(self.w.WaypointsDeepcopy(), [(0, 1, 2), (0, 2, 3), (4, 5, 6)])


	def testRemoveNothing(self):
		lst = [(0, 1, 2), (1, 1, 1), (0, 2, 3), (4, 5, 6)]
		self.w.WaypointsAppendList(lst)
		self.assertEqual(self.w.WaypointsCount(), 4)
		self.w.WaypointsRemoveX(7)
		self.w.WaypointsRemoveY(8)
		self.w.WaypointsRemoveZ(9)
		self.assertEqual(self.w.WaypointsDeepcopy(), [(0, 1, 2), (1, 1, 1), (0, 2, 3), (4, 5, 6)])


	def testTruncate(self):
		lst = [(0, 0, 0), (1, 1, 1), (2, 2, 2), (3, 3, 3), (4, 4, 4)]
		self.w.WaypointsAppendList(lst)
		self.w.WaypointsTruncate(2)
		self.assertEqual(self.w.WaypointsDeepcopy(), [(0, 0, 0), (1, 1, 1), (2, 2, 2)])

	def testTruncateNothing(self):
		lst = [(0, 0, 0), (1, 1, 1), (2, 2, 2), (3, 3, 3), (4, 4, 4)]
		self.w.WaypointsAppendList(lst)
		self.w.WaypointsTruncate(5)
		self.assertEqual(self.w.WaypointsDeepcopy(), [(0, 0, 0), (1, 1, 1), (2, 2, 2), (3, 3, 3), (4, 4, 4)])

	def testClear(self):
		lst = [(0, 0, 0), (1, 1, 1), (2, 2, 2), (3, 3, 3), (4, 4, 4)]
		self.w.WaypointsAppendList(lst)
		self.w.WaypointsClear()
		self.assertEqual(self.w.WaypointsDeepcopy(), [])

	def testRemoveDups(self):
		lst = [(1, 2, 1), (1, 2, 3), (3, 2, 1), (1, 2, 3), (3, 2, 1), (2, 3, 1)]
		self.w.WaypointsAppendList(lst)
		self.w.WaypointsRemoveDups()
		self.assertEqual(self.w.WaypointsDeepcopy(), [(1, 2, 1), (1, 2, 3), (3, 2, 1), (2, 3, 1)])

	def testRemoveNoDups(self):
		lst = [(0, 0, 0), (1, 1, 1), (2, 2, 2), (3, 3, 3), (4, 4, 4)]
		self.w.WaypointsAppendList(lst)
		self.w.WaypointsRemoveDups()
		self.assertEqual(self.w.WaypointsDeepcopy(), [(0, 0, 0), (1, 1, 1), (2, 2, 2), (3, 3, 3), (4, 4, 4)])

if __name__ == '__main__':
    unittest.main()