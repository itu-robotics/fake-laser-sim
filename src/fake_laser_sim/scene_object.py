from turtlesim.msg import Pose
import math
import numpy as np


class Point(object):
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y


class Ray:
    def __init__(self, base: Point, angle: float):
        self.base = base
        self.angle = angle


class SceneObject(object):
    def __init__(self, id: str):
        self.pose = Pose()
        self.id = id

    def intersect(self, ray: Ray) -> float:
        raise NotImplementedError

    def get_id(self) -> str:
        return self.id

    def get_pose(self) -> Pose:
        return self.pose


class Circle(SceneObject):
    def __init__(self, id: str, center: Point, radius: float):
        super().__init__(id)
        self.radius = radius
        self.center = center

    def intersect(self, ray: Ray) -> float:
        x1 = ray.base.x
        y1 = ray.base.y
        x2 = ray.base.x + math.cos(ray.angle)
        y2 = ray.base.y + math.sin(ray.angle)
        x0 = self.center.x
        y0 = self.center.y
        r = self.radius

        a = (x2 - x1) ** 2 + (y2 - y1) ** 2
        b = 2 * ((x2 - x1) * (x1 - x0) + (y2 - y1) * (y1 - y0))
        c = x0 ** 2 + x1 ** 2 - 2 * x0 * x1 + y0 ** 2 + y1 ** 2 - 2 * y0 * y1 - r ** 2

        disc = b ** 2 - 4 * a * c
        if disc < 0:
            return None
        else:
            disc = math.sqrt(disc)

            t1 = (-b + disc) / (2 * a)
            t2 = (-b - disc) / (2 * a)

            t = min(t1, t2)
            if t < 0:
                return None
            else:
                return t


class LineSegment(SceneObject):
    def __init__(self, id: str, start: Point, end: Point):
        super().__init__(id)
        self.start = start
        self.end = end

    def intersect(self, ray: Ray) -> float:
        # This method is slightly modified version of https://gist.github.com/danieljfarrell/faf7c4cafd683db13cbc
        ray_origin = np.array([ray.base.x, ray.base.y], dtype=np.float)
        ray_dir = np.array([math.cos(ray.angle), math.sin(ray.angle)], dtype=np.float)
        point1 = np.array([self.start.x, self.start.y], dtype=np.float)
        point2 = np.array([self.end.x, self.end.y], dtype=np.float)
        
        # Ray-Line Segment Intersection Test in 2D
        # http://bit.ly/1CoxdrG
        v1 = ray_origin - point1
        v2 = point2 - point1
        v3 = np.array([-ray_dir[1], ray_dir[0]])
        d_v2_v3 = np.dot(v2, v3)
        if d_v2_v3 == 0.0:
            return None
        t1 = np.cross(v2, v1) / d_v2_v3
        t2 = np.dot(v1, v3) / d_v2_v3
        if t1 >= 0.0 and 0.0 <= t2 <= 1.0:
            return t1
        return None


class Rectangle(SceneObject):
    def __init__(self, id: str, center: Point, width: float, height: float):
        super().__init__(id)
        self.center = center
        self.width = width
        self.height = height

        corners = self.corners()
        self.segments = [
            LineSegment(corners[0], corners[1]),
            LineSegment(corners[1], corners[2]),
            LineSegment(corners[2], corners[3]),
            LineSegment(corners[3], corners[0])
        ]

    def corners(self) -> list:
        return [
            Point(self.center.x - self.width / 2,
                  self.center.y - self.height / 2),
            Point(self.center.x + self.width / 2,
                  self.center.y - self.height / 2),
            Point(self.center.x + self.width / 2,
                  self.center.y + self.height / 2),
            Point(self.center.x - self.width / 2,
                  self.center.y + self.height / 2)
        ]

    def intersect(self, ray: Ray) -> float:
        min_t = None
        for segment in self.segments:
            t = segment.intersect(ray)
            if t is not None:
                if min_t is None or t < min_t:
                    min_t = t

        return min_t
