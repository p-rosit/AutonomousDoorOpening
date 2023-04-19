#! /usr/bin/env python

import os

import matplotlib.pyplot as plt

import rospy
from sensor_msgs.msg import LaserScan
from jp_exjobb.ros_examples.standard_input import ReadFromStd

class LidarPlotter:
    def __init__(self):
        self.b_scan = None
        self.b_scan_sub = rospy.Subscriber('/b_scan', LaserScan, callback=self.b_scan_callback)
        self.f_scan = None
        self.f_scan_sub = rospy.Subscriber('/f_scan', LaserScan, callback=self.f_scan_callback)

    def b_scan_callback(self, msg):
        self.b_scan = msg
    
    def f_scan_callback(self, msg):
        self.f_scan = msg

    def plot_lidar(self, path, name):
        plot = False
        if name and self.b_scan is not None and self.f_scan is not None:
            _, axs = plt.subplots(2)
            ax1, ax2 = axs
            plt.title('Lidar Data')
            ax1.hist(self.b_scan.ranges)
            ax2.legend(['Back lidar data'])

            ax2.hist(self.f_scan.ranges)
            ax2.legend(['Front lidar data'])

            plt.savefig(os.path.join(path, name) + '.png')
            plt.cla()

        return plot

if __name__ == '__main__':
    rospy.init_node('lidar_plotter')
    lidar_plotter = LidarPlotter()
    read_from_std = ReadFromStd()

    name = ''
    path = './'
    print(
        'Valid commands:\n'
        '\n'
        '   q: Stops the node\n'
        '   a: Reads string from std input, saves the current lidar readings\n'
        '      under that name.\n'
        '   s: Reads a string from std input, this is the path any plots\n'
        '      should be saved to. Default is the current working directory.\n'
    )
    while not rospy.is_shutdown():
        c = read_from_std()

        if c == 'q':
            print('Stopping...')
            break
        elif c == 'a':
            name = read_from_std.get_string()
            plotted = lidar_plotter.plot_lidar(path, name)
            if not plotted:
                print('Did not plot.')
        elif c == 's':
            path = read_from_std.get_string()
        else:
            print('"%c" is not a recognized command.' % c)
