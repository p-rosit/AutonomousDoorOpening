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
        time = np.linspace(0, self.time, len(f))

        fig, axs = plt.subplots(1)
        plt.title('Wrench Magnitude')
        axs.plot(time, f)
        axs.plot(time, t)
        axs.legend(['Force', 'Torque'])
        axs.set_xlabel('Time (s)')
        axs.set_ylabel('Magnitude')
        axs.set_ylim(bottom=0)
        plt.savefig(os.path.join(path, name + '_wrench_magnitude') + '.png')
        plt.cla()

        fig, axs = plt.subplots(2)
        plt.suptitle('Wrench')
        ax1, ax2 = axs
        for f in self.w[:3]:
            ax1.plot(time, f)
        ax1.legend(['force x', 'force y', 'force z'])
        ax1.set_ylabel('Force (N)')
        for t in self.w[3:]:
            ax2.plot(time, t)
        ax2.legend(['torque x', 'torque y', 'torque z'])
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Torque (Nm)')
        plt.savefig(os.path.join(path, name + '_wrench_coords') + '.png')
        plt.cla()

        f, t = self.filter_mag

        fig, axs = plt.subplots(1)
        plt.title('Filter Magnitude')
        axs.plot(f)
        axs.plot(t)
        axs.legend(['Force', 'Torque'])
        axs.set_xlabel('Time (s)')
        axs.set_ylabel('Magnitude')
        axs.set_ylim(bottom=0)
        plt.savefig(os.path.join(path, name + '_filter_magnitude') + '.png')
        plt.cla()

        fig, axs = plt.subplots(2)
        plt.suptitle('Filter')
        ax1, ax2 = axs
        for f in self.f[:3]:
            ax1.plot(f)
        ax1.legend(['force x', 'force y', 'force z'])
        ax1.set_ylabel('Force (N)')
        for t in self.f[3:]:
            ax2.plot(t)
        ax2.legend(['torque x', 'torque y', 'torque z'])
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Torque (Nm)')
        plt.savefig(os.path.join(path, name + '_filter_coords') + '.png')
        plt.cla()
        print('Wrench plotted in time domain with name "%s".' % name)

    def plot_wrench_frequency_domain(self, path, name):
        f, t = self.wrench_mag
        f, t = np.array(f), np.array(t)
        f = np.abs(np.fft.fft(np.array(f - f.mean())))
        t = np.abs(np.fft.fft(np.array(t - t.mean())))
        freq = np.linspace(0, 250, len(f) // 2)

        fig, axs = plt.subplots(1)
        plt.title('Wrench Magnitude Fourier Transform')
        axs.plot(freq, f[:len(f)//2])
        axs.plot(freq, t[:len(f)//2])
        axs.legend(['Force', 'Torque'])
        axs.set_xlabel('Frequency (Hz)')
        axs.set_ylabel('Magnitude')
        axs.set_ylim(bottom=0)
        plt.savefig(os.path.join(path, name + '_wrench_magnitude_freq') + '.png')
        plt.cla()

        fig, axs = plt.subplots(2)
        plt.suptitle('Wrench Fourier Transform')
        ax1, ax2 = axs
        for f in self.w[:3]:
            f = np.array(f)
            f = np.abs(np.fft.fft(f - f.mean()))
            ax1.plot(freq, f[:len(f)//2])
        ax1.legend(['force x', 'force y', 'force z'])
        ax1.set_ylabel('Force (N)')
        ax1.set_ylim(bottom=0)
        for t in self.w[3:]:
            t = np.array(t)
            t = np.abs(np.fft.fft(t - t.mean()))
            ax2.plot(freq, t[:len(f)//2])
        ax2.legend(['torque x', 'torque y', 'torque z'])
        ax2.set_xlabel('Frequency (Hz)')
        ax2.set_ylabel('Torque (Nm)')
        ax2.set_ylim(bottom=0)
        plt.savefig(os.path.join(path, name + '_wrench_coords_freq') + '.png')
        plt.cla()

        f, t = self.filter_mag
        f, t = np.array(f), np.array(t)
        f = np.abs(np.fft.fft(f - f.mean()))
        t = np.abs(np.fft.fft(t - t.mean()))

        fig, axs = plt.subplots(1)
        plt.title('Filter Magnitude Fourier Transform')
        axs.plot(freq, f[:len(f)//2])
        axs.plot(freq, t[:len(f)//2])
        axs.legend(['Force', 'Torque'])
        axs.set_xlabel('Frequency (Hz)')
        axs.set_ylabel('Magnitude (N)')
        axs.set_ylim(bottom=0)
        plt.savefig(os.path.join(path, name + '_filter_magnitude_freq') + '.png')
        plt.cla()

        fig, axs = plt.subplots(2)
        plt.suptitle('Filter Fourier Transform')
        ax1, ax2 = axs
        for f in self.f[:3]:
            f = np.array(f)
            f = np.abs(np.fft.fft(f - f.mean()))
            ax1.plot(freq, f[:len(f)//2])
        ax1.legend(['force x', 'force y', 'force z'])
        ax1.set_ylabel('Force (N)')
        ax1.set_ylim(bottom=0)
        for t in self.f[3:]:
            t = np.array(t)
            t = np.abs(np.fft.fft(t - t.mean()))
            ax2.plot(freq, t[:len(f)//2])
        ax2.legend(['torque x', 'torque y', 'torque z'])
        ax2.set_xlabel('Frequency (Hz)')
        ax2.set_ylabel('Torque (N)')
        ax2.set_ylim(bottom=0)
        plt.savefig(os.path.join(path, name + '_filter_coords_freq') + '.png')
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
        '      wrench are recorded for that amount of time. A string is then\n'
        '      read from std input, both the wrench and the filtered wrench\n'
        '      are plotted and saved with that file name.\n'
        '   s: Reads a string from std input, this is the path any plots\n'
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
            print('Plot name: ', end='', flush=True)
            name = read_from_std.get_string()
            plotted = wrench_plotter.plot_wrench(path, name)
            if not plotted:
                print('Did not plot. Either name was empty or recording has not been made.')
        elif c == 's':
            print('Path: ', end='', flush=True)
            path = read_from_std.get_string()
        else:
            print('"%c" is not a recognized command.' % c)
