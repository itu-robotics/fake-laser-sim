#!/usr/bin/env python3
import rospy
from fake_laser_sim.srv import SpawnShape, SpawnShapeRequest, SpawnShapeResponse
from fake_laser_sim import Point, Pose, Circle, Rectangle, Ray, LineSegment, LaserSensor
from sensor_msgs.msg import LaserScan
from fake_laser_sim.conversions import create_circle, create_rectangle, create_line_segment
import math
import numpy as np


class EnvServerNode:
    __req = SpawnShapeRequest()
    create_obj_map = {
        __req.shape.TYPE_CIRCLE: create_circle,
        __req.shape.TYPE_RECTANGLE: create_rectangle,
        __req.shape.TYPE_LINE: create_line_segment
    }

    def __init__(self):
        self.shapes = []
        self.laser_frame_id = rospy.get_param("~laser/frame_id", "turtle112")
        self.laser_rays = rospy.get_param("~laser/rays", 60)
        self.laser_start_angle = rospy.get_param(
            "~laser/start_angle", -math.pi)
        self.laser_end_angle = rospy.get_param("~laser/end_angle", math.pi)
        self.laser_noise_stddev = rospy.get_param("~laser/noise_std_dev", 0.01)

        self.laser = LaserSensor(
            self.laser_frame_id, self.laser_rays, self.laser_start_angle, self.laser_end_angle)
        self.laser_pub = rospy.Publisher(
            "scan", LaserScan, queue_size=10)
        self.pose_sub = rospy.Subscriber(
            "pose", Pose, self.pose_callback)

        self.shape_srv = rospy.Service(
            "spawn_shape", SpawnShape, self.spawn_handler)

    def pose_callback(self, msg: Pose):
        self.laser.set_pose(msg)
        self.laser.scan(self.shapes)
        laser_msg = self.laser.get_scan_msg(rospy.Time.now())

        if self.laser_noise_stddev != 0.0:
            noised = np.random.normal(np.array(laser_msg.ranges), self.laser_noise_stddev)
            laser_msg.ranges = noised.tolist()

        self.laser_pub.publish(laser_msg)

    def spawn_handler(self, req: SpawnShapeRequest):
        def on_fail(msg: str) -> SpawnShapeResponse:
            rospy.logerr(msg)
            return SpawnShapeResponse(success=False, message=msg)

        def on_success(msg: str) -> SpawnShapeResponse:
            rospy.loginfo(msg)
            return SpawnShapeResponse(success=True, message=msg)

        shape = EnvServerNode.create_obj_map[req.shape.type](req.id, req.shape)
        if shape is None:
            return on_fail(f"Failed to create {req.id}")

        self.shapes.append(shape)
        return on_success(f"Created {req.id}")


if __name__ == '__main__':
    rospy.init_node('env_server_node')
    EnvServerNode()
    rospy.spin()
