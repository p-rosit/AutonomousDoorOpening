#! /usr/bin/env python

import sys
from select import select
import termios
import tty

import rospy
from geometry_msgs.msg import WrenchStamped

class ReadFromStd:
    def __call__(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def get_float(self):
        try:
            num = float(self.get_string())
        except ValueError:
            raise ValueError('Expected valid float input, got "%s".' % num)
        
        return num

    def get_string(self):
        string = ""
        while True:
            ch = self()
            if ch == '\r':
                break

            if ch == '\x7f':
                if string != "":
                    string = string[:-1]
                print('\b \b', end='', flush=True)
            else:
                string += ch
                print(ch, end='', flush=True)
        
        print('')
        return string

class WrenchPlotter:
    def __init__(self):
        self.hz = 30
        self.rate = rospy.Rate(self.hz)

        self.wrench = None
        self.filter = None
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
        if self.wrench is not None and self.filter is not None:
            self.w = ([], [], [], [], [], [])
            self.f = ([], [], [], [], [], [])
            self.time = time

            ind = 0
            while ind < self.hz * self.time:
                [
                    ls.append(val) for ls, val in zip(self.w,
                        (
                            self.wrench.force.x,
                            self.wrench.force.y,
                            self.wrench.force.z,
                            self.wrench.torque.x,
                            self.wrench.torque.y,
                            self.wrench.torque.z,
                        )
                    )
                ]
                [
                    ls.append(val) for ls, val in zip(self.f,
                        (
                            self.filter.force.x,
                            self.filter.force.y,
                            self.filter.force.z,
                            self.filter.torque.x,
                            self.filter.torque.y,
                            self.filter.torque.z,
                        )
                    )
                ]
                ind += 1
                self.rate.sleep()

    def plot_wrench_time_domain(name):
        pass

    def plot_wrench_frequency_domain(name):
        pass

if __name__ == '__main__':
    rospy.init_node('wrench_plotter')
    wrench_plotter = WrenchPlotter()
    read_from_std = ReadFromStd()

    reading_time = False
    has_recorded = False
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
            wrench_plotter.plot_wrench_time_domain(name)
            wrench_plotter.plot_wrench_frequency_domain(name)
        elif c == 'd':
            print('Path: ', end='', flush=True)
            path = read_from_std.get_string()
