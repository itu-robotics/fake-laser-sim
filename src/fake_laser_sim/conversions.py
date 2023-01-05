from fake_laser_sim import Rectangle, Point, Circle, LineSegment
from fake_laser_sim.msg import Shape
import math


def create_rectangle(id: str, shape: Shape) -> Rectangle:
    if shape.width is None or shape.height is None:
        return None

    return Rectangle(
        id,
        Point(
            shape.center.x,
            shape.center.y
        ),
        shape.width,
        shape.height
    )


def create_circle(id: str, shape: Shape) -> Circle:
    if shape.radius is None:
        return None

    if shape.center.x is None or shape.center.y is None:
        return None

    return Circle(
        id,
        Point(
            shape.center.x,
            shape.center.y
        ),
        shape.radius
    )


def create_line_segment(id: str, shape: Shape) -> LineSegment:
    angle = shape.center.theta
    if angle is None:
        return None

    endpoint = Point(
        shape.center.x + math.cos(angle) * shape.length,
        shape.center.y + math.sin(angle) * shape.length
    )

    return LineSegment(
        id,
        Point(
            shape.center.x,
            shape.center.y
        ),
        Point(
            endpoint.x,
            endpoint.y
        )
    )
