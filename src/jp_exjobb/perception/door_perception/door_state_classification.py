
import rospy
from sensor_msgs.msg import LaserScan
import tf2_ros
from tf2_geometry_msgs import PoseStamped

import numpy as np
from scipy.spatial.distance import cdist

class DoorStateClassifier:
    def __init__(self, door_bb, open_threshold, closed_threshold):
        self.open_threshold = open_threshold
        self.closed_threshold = closed_threshold
        self.door_state_known = False
        self.door_open = False
        self.door_fill = -1.0

        self.bb_frame = door_bb.getProperty('skiros:FrameId').value
        self.bb_size = door_bb.getProperty('skiros:Size').value / 2
        self.bb_sizex = door_bb.getProperty('skiros:SizeX').value / 2
        self.bb_sizey = door_bb.getProperty('skiros:SizeY').value / 2

        self.buffer = tf2_ros.Buffer()  # type: any
        self.tf_listener = tf2_ros.TransformListener(self.buffer)

        self.sub = rospy.Subscriber('/scan', LaserScan, callback=self.check_lidar)

    def unregister(self):
        self.sub.unregister()

    def get_door_state(self):
        return self.door_state_known, self.door_open
    
    def get_filled_value(self):
        return self.door_fill / 2

    def check_lidar(self, msg):
        frame, pts = self.lidar2points(msg)
        pts = self.transform_lidar(frame, pts)

        x, y = pts
        x_in_door = (-self.bb_sizex < x) & (x < self.bb_sizex)
        y_in_door = (-self.bb_sizey < y) & (y < self.bb_sizey)
        x_by_door = (-self.bb_sizex - self.bb_size < x) & (x < self.bb_sizex + self.bb_size)

        door_frame_pts = pts[:, (x_by_door & y_in_door) & ~(x_in_door & y_in_door)]
        left_pts = (door_frame_pts[0] < 0).sum()
        right_pts = (door_frame_pts[0] > 0).sum()
        door_pts = pts[:, x_in_door & y_in_door]

        # TODO: RanSaC inside bounding box to detect line and throw away points not on door
        # Problem with that kind of outlier rejection: doors that are not straight, elevator door...

        if door_frame_pts.shape[1] < 2 or left_pts == 0 or right_pts == 0:
            return

        if door_pts.shape[1] < 5:
            self.door_state_known = True
            self.door_fill = 0.0
            self.door_open = True
            return

        dists = cdist(door_pts.T, door_pts.T)
        dists = dists.reshape(-1)

        dists[dists == 0.0] = np.inf
        dist = dists.min()

        filled_length = dist * door_pts.shape[1] / self.bb_sizex
        self.door_fill = filled_length

        if self.closed_threshold < filled_length:
            self.door_open = False
            self.door_state_known = True
            return
        
        if filled_length < self.open_threshold:
            self.door_open = True
            self.door_state_known = True
            return
        
        self.door_state_known = False

    def transform_lidar(self, frame, pts):
        x, y = pts
        for i in range(pts.shape[1]):
            pose = PoseStamped()
            pose.header.frame_id = frame
            pose.header.stamp = rospy.Time(0)
            pose.pose.position.x = x[i]
            pose.pose.position.y = y[i]
            
            pose = self.buffer.transform(pose, self.bb_frame, rospy.Duration(1))
            
            x[i] = pose.pose.position.x
            y[i] = pose.pose.position.y
        
        return pts

    def lidar2points(self, msg):
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        lengths = np.array(msg.ranges)
        angles = np.linspace(angle_min, angle_max, num=lengths.shape[0])
        x = lengths * np.cos(angles)
        y = lengths * np.sin(angles)
        pts = np.zeros((2, x.shape[0]))
        pts[0] = x #ting 1
        pts[1] = y #ting 2

        return msg.header.frame_id, pts