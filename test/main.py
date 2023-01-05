import unittest
import fake_laser_sim
import math
from turtlesim.msg import Pose


class TestIntersection(unittest.TestCase):
    def test_intersection(self):
        circle = fake_laser_sim.Circle("circle1", fake_laser_sim.Point(4, 4), 1)
        ray = fake_laser_sim.Ray(fake_laser_sim.Point(0, 0), math.radians(45.0))

        res = circle.intersect(ray)
        self.assertTrue(res is not None)
        self.assertAlmostEqual(res, math.hypot(circle.center.x, circle.center.y) - circle.radius, places=5)


    def test_line_intersection(self):
        line = fake_laser_sim.LineSegment("ls", fake_laser_sim.Point(2, 0), fake_laser_sim.Point(0, 2))
        ray = fake_laser_sim.Ray(fake_laser_sim.Point(0, 0), math.radians(45.0))

        res = line.intersect(ray)
        self.assertTrue(res is not None)
        self.assertAlmostEqual(res, math.sqrt(2.0), places=5)


        line = fake_laser_sim.LineSegment("ls", fake_laser_sim.Point(4, 0), fake_laser_sim.Point(4, 6))
        ray = fake_laser_sim.Ray(fake_laser_sim.Point(2, 5), math.radians(-45.0))
        res = line.intersect(ray)
        self.assertTrue(res is not None)
        self.assertAlmostEqual(res, 2.0 * math.sqrt(2.0), places=5)


if __name__ == '__main__':
    unittest.main()