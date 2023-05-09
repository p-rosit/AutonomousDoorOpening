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
        self.addParam('time_limit', 10.0, ParamTypes.Required)

class watch_for_door(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(WatchForDoor(), self.__class__.__name__)
    
    def onInit(self):
        return True

    def preStart(self):
        return True

    def check_lidar(self, msg):
        pass

    def run(self):
        left_corner, right_corner = self.infer_corners()
        
        return self.success('No obstruction was detected.')

    def infer_corners(self):
        return 0, 0

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
