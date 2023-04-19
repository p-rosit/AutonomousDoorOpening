#! /usr/bin/env python

import os

import numpy as np
import matplotlib.pyplot as plt

import rospy
from geometry_msgs.msg import WrenchStamped
from jp_exjobb.ros_examples.standard_input import ReadFromStd

class WrenchPlotter:
    def __init__(self):
        self.hz = 30
        self.rate = rospy.Rate(self.hz)

        self.wrench = None
        self.filter = None
        self.wrench_mag = None
        self.filter_mag = None
        self.time = 0
        self.w = ([], [], [], [], [], [])
        self.f = ([], [], [], [], [], [])
        self.wrench_sub = rospy.Subscriber('/wrench', WrenchStamped, callback=self.wrench_listen)
        self.filter_sub = rospy.Subscriber('/cartesian_compliance_controller/ft_sensor_wrench', WrenchStamped, callback=self.filter_listen)

    def wrench_listen(self, msg):
        self.wrench = msg.wrench

    def filter_listen(self, msg):
        self.filter = msg.wrench

    def record_wrench(self, time):
            self.w = ([], [], [], [], [], [])
            self.f = ([], [], [], [], [], [])
            self.time = time

            ind = 0
            while ind < self.hz * self.time:
                if self.wrench is not None:
                    [
                        ls.append(val) for ls, val in zip(self.w,
                            (
                                self.wrench.force.x,
                                self.wrench.force.y,
                                self.wrench.force.z,
                                self.wrench.torque.x,
                                self.wrench.torque.y,
                                self.wrench.torque.z
                            )
                        )
                    ]
                if self.filter is not None:
                    [
                        ls.append(val) for ls, val in zip(self.f,
                            (
                                self.filter.force.x,
                                self.filter.force.y,
                                self.filter.force.z,
                                self.filter.torque.x,
                                self.filter.torque.y,
                                self.filter.torque.z
                            )
                        )
                    ]
                ind += 1
                self.rate.sleep()

    def compute_magnitude(self):
        fx, fy, fz, tx, ty, tz = self.w
        self.wrench_mag = (
            [np.sqrt(x**2 + y**2 + z**2) for x, y, z in zip(fx, fy, fz)],
            [np.sqrt(x**2 + y**2 + z**2) for x, y, z in zip(tx, ty, tz)]
        )
        fx, fy, fz, tx, ty, tz = self.f
        self.filter_mag = (
            [np.sqrt(x**2 + y**2 + z**2) for x, y, z in zip(fx, fy, fz)],
            [np.sqrt(x**2 + y**2 + z**2) for x, y, z in zip(tx, ty, tz)]
        )

    def plot_wrench(self, path, name):
        plot = False
        if self.w[0] and name:
            plot = True

            self.compute_magnitude()
            self.plot_wrench_time_domain(path, name)
            self.plot_wrench_frequency_domain(path, name)
        
        return plot

    def plot_wrench_time_domain(self, path, name):
        f, t = self.wrench_mag

        fig, axs = plt.subplots(1)
        plt.title('Wrench Magnitude')
        axs.plot(f)
        axs.plot(t)
        axs.legend(['Force', 'Torque'])
        plt.savefig(os.path.join(path, 'wrench_magnitude_' + name) + '.png')
        plt.cla()

        fig, axs = plt.subplots(2)
        plt.title('Wrench')
        ax1, ax2 = axs
        for f in self.w[:3]:
            ax1.plot(f)
        ax1.legend(['force x', 'force y', 'force z'])
        for t in self.w[3:]:
            ax2.plot(t)
        ax2.legend(['torque x', 'torque y', 'torque z'])
        plt.savefig(os.path.join(path, 'wrench_coords_' + name) + '.png')
        plt.cla()

        f, t = self.filter_mag

        fig, axs = plt.subplots(1)
        plt.title('Filter Magnitude')
        axs.plot(f)
        axs.plot(t)
        axs.legend(['Force', 'Torque'])
        plt.savefig(os.path.join(path, 'filter_magnitude_' + name) + '.png')
        plt.cla()

        fig, axs = plt.subplots(2)
        plt.title('Filter')
        ax1, ax2 = axs
        for f in self.f[:3]:
            ax1.plot(f)
        ax1.legend(['force x', 'force y', 'force z'])
        for t in self.f[3:]:
            ax2.plot(t)
        ax2.legend(['torque x', 'torque y', 'torque z'])
        plt.savefig(os.path.join(path, 'filter_coords_' + name) + '.png')
        plt.cla()
        print('Wrench plotted in time domain with name "%s".' % name)

    def plot_wrench_frequency_domain(self, path, name):
        f, t = self.wrench_mag
        f = np.abs(np.fft.fft(np.array(f - f.mean())))
        t = np.abs(np.fft.fft(np.array(t - t.mean())))

        fig, axs = plt.subplots(1)
        plt.title('Wrench Magnitude Fourier Transform')
        axs.plot(f)
        axs.plot(t)
        axs.legend(['Force', 'Torque'])
        plt.savefig(os.path.join(path, 'wrench_magnitude_freq_' + name) + '.png')
        plt.cla()

        fig, axs = plt.subplots(2)
        plt.title('Wrench Fourier Transform')
        ax1, ax2 = axs
        for f in self.w[:3]:
            f = np.abs(np.fft.fft(f - f.mean()))
            ax1.plot(f)
        ax1.legend(['force x', 'force y', 'force z'])
        for t in self.w[3:]:
            t = np.abs(np.fft.fft(t - t.mean()))
            ax2.plot(t)
        ax2.legend(['torque x', 'torque y', 'torque z'])
        plt.savefig(os.path.join(path, 'wrench_coords_freq_' + name) + '.png')
        plt.cla()

        f, t = self.filter_mag
        f = np.abs(np.fft.fft(np.array(f - f.mean())))
        t = np.abs(np.fft.fft(np.array(t - t.mean())))

        fig, axs = plt.subplots(1)
        plt.title('Filter Magnitude Fourier Transform')
        axs.plot(f)
        axs.plot(t)
        axs.legend(['Force', 'Torque'])
        plt.savefig(os.path.join(path, 'filter_magnitude_freq_' + name) + '.png')
        plt.cla()

        fig, axs = plt.subplots(2)
        plt.title('Filter Fourier Transform')
        ax1, ax2 = axs
        for f in self.f[:3]:
            f = np.abs(np.fft.fft(f - f.mean()))
            ax1.plot(f)
        ax1.legend(['force x', 'force y', 'force z'])
        for t in self.f[3:]:
            t = np.abs(np.fft.fft(t - t.mean()))
            ax2.plot(t)
        ax2.legend(['torque x', 'torque y', 'torque z'])
        plt.savefig(os.path.join(path, 'filter_coords_freq_' + name) + '.png')
        plt.cla()
        print('Wrench plotted in frequency domain with name "%s".' % name)

if __name__ == '__main__':
    rospy.init_node('wrench_plotter')
    wrench_plotter = WrenchPlotter()
    read_from_std = ReadFromStd()

    name = ''
    path = './'
    print(
        'Valid commands:\n'
        '\n'
        '   q: Stops the node\n'
        '   a: Reads a float from std input, both the wrench and filtered\n'
        '      wrench are recorded for that amount of time.\n'
        '   s: Reads a string from std input, both the wrench and the\n'
        '      filtered wrench are plotted and saved with that file name.\n'
        '   d: Reads a string from std input, this is the path any plots\n'
        '      should be saved to. Default is the current working directory.\n'
    )
    while not rospy.is_shutdown():
        c = read_from_std()

        if c == 'q':
            print('Stopping...')
            break
        elif c == 'a':
            print('Recording length (s): ', end='', flush=True)
            time = read_from_std.get_float()
            wrench_plotter.record_wrench(time)
            print('Recorded wrench for %f seconds.' % time)
        elif c == 's':
            print('Plot name: ', end='', flush=True)
            name = read_from_std.get_string()
            plotted = wrench_plotter.plot_wrench(path, name)
            if not plotted:
                print('Did not plot. Either name was empty or recording has not been made.')
        elif c == 'd':
            print('Path: ', end='', flush=True)
            path = read_from_std.get_string()
        else:
            print('"%c" is not a recognized command.' % c)
