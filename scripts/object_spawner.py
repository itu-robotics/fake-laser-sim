#!/usr/bin/env python3
import rospy
from fake_laser_sim.srv import SpawnShape, SpawnShapeRequest, SpawnShapeResponse
from fake_laser_sim.msg import Shape
from fake_laser_sim import Point, Pose, Circle, Rectangle, Ray, LineSegment, LaserSensor
import math

class SpawnerNode:
    def __init__(self):
        self.shape_srv = rospy.ServiceProxy(
            "spawn_shape", SpawnShape)

        self.shape_srv.wait_for_service()

        self.spawn_walls()

        self.spawn_obstacles()

    def spawn_obstacles(self):
        req = SpawnShapeRequest(id="obstacle_1", shape=Shape(type=Shape.TYPE_CIRCLE, center=Pose(x=8.0, y=5, theta=0), radius=1.0))
        self.shape_srv.call(req)

        # req = SpawnShapeRequest(id="obstacle_2", shape=Shape(type=Shape.TYPE_LINE, center=Pose(x=8.0, y=8.0, theta=math.pi), length=10.0))
        # self.shape_srv.call(req)


    def generate_walls(self) -> list:
        return [
            Shape(type=Shape.TYPE_LINE, center=Pose(x=0, y=0, theta=0), length=10.0),
            Shape(type=Shape.TYPE_LINE, center=Pose(x=10, y=0, theta=math.pi / 2), length=10.0),
            Shape(type=Shape.TYPE_LINE, center=Pose(x=10, y=10, theta=-math.pi), length=10.0),
            Shape(type=Shape.TYPE_LINE, center=Pose(x=0, y=10, theta=-math.pi / 2), length=10.0)
        ]


    def spawn_walls(self):
        for wall in self.generate_walls():
            req = SpawnShapeRequest(id=f"wall_{wall.center.x}_{wall.center.y}", shape=wall)
            res = self.shape_srv.call(req)
            if not res.success:
                rospy.logerr(res.message)
            else:
                rospy.loginfo(res.message)

if __name__ == '__main__':
    rospy.init_node('spawner_node')
    SpawnerNode()
    # rospy.spin()