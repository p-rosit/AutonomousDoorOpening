from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_skill.core.skill import SkillBase, SkillDescription, Sequential, ParallelFf
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

import rospy
from sensor_msgs.msg import LaserScan
import tf2_ros
from tf2_geometry_msgs import PoseStamped

import numpy as np
import matplotlib.pylab as plt

class WatchForDoor(SkillDescription):
    def createDescription(self):
        self.addParam('Region Transition', Element('scalable:RegionTransition'), ParamTypes.Required)
        self.addParam('Region Bounding Box', Element('scalable:RegionBB'), ParamTypes.Inferred)
        self.addParam('Time Limit (s)', 3600.0, ParamTypes.Required)
        self.addParam('Door Open Threshold', 0.3, ParamTypes.Required)
        self.addParam('Door Closed Threshold', 0.8, ParamTypes.Required)

        self.addPreCondition(self.getRelationCond('DoorHasBB', 'skiros:hasA', 'Region Transition', 'Region Bounding Box', True))

class watch_for_door(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(WatchForDoor(), self.__class__.__name__)
    
    def onInit(self):
        self.hz = 50
        self.rate = rospy.Rate(self.hz)
        self.running = False
        self.sub = rospy.Subscriber('/scan', LaserScan, callback=self.check_lidar)

        self.buffer = tf2_ros.Buffer()  # type: any
        self.tf_listener = tf2_ros.TransformListener(self.buffer)
        
        return True

    def preStart(self):
        self.door_open = False
        self.door_intermediate = False
        self.door_bb = self.params['Region Bounding Box'].value  # big boobed door
        self.open_threshold = self.params['Door Open Threshold'].value
        self.closed_threshold = self.params['Door Closed Threshold'].value

        self.bb_frame = self.door_bb.getProperty('skiros:FrameId').value
        self.bb_sizex = self.door_bb.getProperty('skiros:SizeX').value / 2
        self.bb_sizey = self.door_bb.getProperty('skiros:SizeY').value / 2

        self.preempted = False
        self.running = True
        return True

    def check_lidar(self, msg):
        if self.running:
            frame, pts = self.lidar2points(msg)
            pts = self.transform_lidar(frame, pts)

            # print(pts)

            x, y = pts
            x_in_door = (-self.bb_sizex < pts[0]) & (pts[0] < self.bb_sizex)
            y_in_door = (-self.bb_sizey < pts[1]) & (pts[1] < self.bb_sizey)

            x = x[x_in_door & y_in_door]
            # y = y[x_in_door & y_in_door]

            # TODO: RanSaC inside bounding box to detect line and throw away points not on door
            # Problem with that kind of outlier rejection: doors that are not straight, elevator door...

            if x.shape[0] < 5:
                return

            print('shape', pts.shape)

            pts = pts[0].reshape((-1, 1))
            dists = np.abs(pts - pts.T)

            dists[dists == 0.0] = np.inf
            dists = np.sort(dists)
            dist = dists[dists.shape[0] // 2]

            print('dist', dist)

            filled_area = dist * pts.shape[0] / self.bb_sizex

            print('area', filled_area)

            if self.closed_threshold < filled_area:
                self.door_open = False
                self.door_intermediate = False
                return
            
            if filled_area < self.open_threshold:
                self.door_open = True
                self.door_intermediate = False
                return
            
            # self.door_intermediate = True

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

    def onPreempt(self):
        self.preempted = True
        return self.fail('Watching preempted.', -1)

    def run(self):
        time_limit = self.params['Time Limit (s)'].value

        ind = 0
        while ind < self.hz * time_limit and not self.preempted:
            # if not self.door_intermediate:
            if self.door_open:
                self.status = 'Door gay'
            else:
                self.status = 'Door still in closet'
            # else:
            #     self.status = 'Door sexuality unknown'
            ind +=1
            self.rate.sleep()
        
        return self.success('Door stayed open during skill.')

    def onEnd(self):
        self.running = False
        return True

class watchwatchwatch(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(WatchForDoor(), self.__class__.__name__)

    def onInit(self):
        self.f_sub = rospy.Subscriber('/f_scan', LaserScan, callback=self.f_scan)
        self.b_sub = rospy.Subscriber('/b_scan', LaserScan, callback=self.b_scan)

        self.buffer = tf2_ros.Buffer()  # type: any
        self.tf_listener = tf2_ros.TransformListener(self.buffer)

        return True

    def preStart(self):
        self.f = None
        self.b = None
        return True

    def f_scan(self, msg):
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        lengths = np.array(msg.ranges)
        angles = np.linspace(angle_min, angle_max, num=lengths.shape[0])
        x = lengths * np.cos(angles)
        y = lengths * np.sin(angles)

        frame = msg.header.frame_id
        
        for i in range(x.shape[0]):
            x[i], y[i] = self.transform(frame, x[i], y[i])

        self.f = (x, y)

    def b_scan(self, msg):
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        lengths = np.array(msg.ranges)
        angles = np.linspace(angle_min, angle_max, num=lengths.shape[0])
        x = lengths * np.cos(angles)
        y = lengths * np.sin(angles)

        frame = msg.header.frame_id
        
        for i in range(x.shape[0]):
            x[i], y[i] = self.transform(frame, x[i], y[i])

        self.b = (x, y)
    
    def transform(self, frame, x, y):
        pose = PoseStamped()
        pose.header.frame_id = frame
        pose.header.stamp = rospy.Time(0)
        pose.pose.position.x = x
        pose.pose.position.y = y
        
        pose = self.buffer.transform(pose, 'map', rospy.Duration(1))
        
        x = pose.pose.position.x
        y = pose.pose.position.y
        return x,y
                

    def run(self):
        while self.f is None or self.b is None:
            rospy.sleep(0.5)

        f_x, f_y = self.f
        b_x, b_y = self.b

        plt.cla()

        plt.axis('equal')

        plt.plot(f_x, f_y, '*')
        plt.plot(b_x, b_y, '*')
        plt.xlim(35, 50)
        plt.ylim(0, 10)
        
        plt.savefig('/home/duploproject/penis.png')
        
        return self.success(':)')
