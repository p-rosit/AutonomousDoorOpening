#! /usr/bin/env python

import rospy
import matplotlib.pyplot as plt
import numpy as np

from sensor_msgs.msg import LaserScan

class LidarReader():
    def __init__(self):
        self.b_scan = None
        self.b_scan_sub = rospy.Subscriber('/b_scan', LaserScan, callback=self.b_scan_callback)
        self.f_scan = None
        self.f_scan_sub = rospy.Subscriber('/f_scan', LaserScan, callback=self.f_scan_callback)

        self.nbr_of_msgs = 0
        self.start_time = rospy.Time.now()
        self.interval = rospy.Duration(30)

    def b_scan_callback(self, msg):
        self.b_scan = msg

    def f_scan_callback(self, msg):
        self.f_scan = msg
    
    def plot_lidar(self):
        plt.cla()

        f_angle_min = self.f_scan.angle_min
        f_angle_max = self.f_scan.angle_max
        f_values = np.array(self.f_scan.ranges)
        f_angles = np.linspace(f_angle_min, f_angle_max, num=f_values.shape[0])
        f_x = f_values * np.cos(f_angles)
        f_y = f_values * np.sin(f_angles)
        
        plt.axis('equal')
        plt.plot(f_x, f_y, '*')
        
        b_angle_min = self.b_scan.angle_min
        b_angle_max = self.b_scan.angle_max
        b_values = np.array(self.b_scan.ranges)
        b_angles = np.linspace(b_angle_min, b_angle_max, num=b_values.shape[0])
        b_x = b_values * np.cos(b_angles)
        b_y = b_values * np.sin(b_angles)

        plt.plot(-b_x, -b_y, '*')
        
        plt.draw()
        plt.pause(0.001)


if __name__ == '__main__':
    rospy.init_node('lidar_msgs')
    read = LidarReader()
    rate = rospy.Rate(10)

    while True:
        rate.sleep()
        read.plot_lidar()

