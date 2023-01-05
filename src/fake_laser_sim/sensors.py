from turtlesim.msg import Pose
from sensor_msgs.msg import LaserScan
from fake_laser_sim import Point, Pose, Circle, Rectangle, Ray, LineSegment
import rospy
import copy
import angles

class Sensor(object):
    def __init__(self):
        pass


class LaserSensor(object):
    def __init__(self, frame_id: str, n_rays: int, start_angle: float, end_angle: float):
        self.pose = Pose()
        self.n_rays = n_rays
        self.start_angle = start_angle
        self.end_angle = end_angle
        self.scan_angle = end_angle - start_angle
        self.angle_increment = self.scan_angle / n_rays
        self.ranges = [0.0] * n_rays
        self.frame_id = frame_id

    def set_pose(self, pose: Pose):
        self.pose = pose

    def __get_min_intersection(self, ray: Ray, objects: list) -> float:
        min_dist = float('inf')
        for obj in objects:
            dist = obj.intersect(ray)
            if dist is not None and dist < min_dist:
                min_dist = dist
        return min_dist

    def scan(self, objects: list):
        for i in range(self.n_rays):
            angle = self.start_angle + i * self.angle_increment
            ray = Ray(self.pose, angles.normalize_angle_positive(self.pose.theta + angle))
            self.ranges[i] = self.__get_min_intersection(ray, objects)

    def get_scan_msg(self, time: rospy.Time) -> LaserScan:
        scan = LaserScan()
        scan.angle_min = self.start_angle
        scan.angle_increment = self.angle_increment
        scan.angle_max = self.end_angle
        scan.time_increment = 0.001
        # scan.intensities = [1.0] * self.n_rays
        # scan.scan_time = scan.time_increment * self.n_rays
        scan.header.frame_id = self.frame_id
        scan.header.stamp = time
        scan.range_min = 0.01
        scan.range_max = 20.0
        r = copy.deepcopy(self.ranges)
        scan.ranges = r
        return scan
