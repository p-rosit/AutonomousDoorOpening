#! /usr/bin/env python

import rospy
from geometry_msgs.msg import WrenchStamped

class Counter():
    def init(self):
        self.wrench_sub = rospy.Subscriber('/wrench', WrenchStamped, callback=self.callback_counter)
        self.nbr_of_msgs = 0
        self.start_time = rospy.Time.now()
        self.interval = rospy.Duration(30)
        

    def start_measuring(self):
        self.start_time = rospy.Time.now()
        self.nbr_of_msgs = 0

    def set_interval(self, time):
        self.interval = rospy.Duration(time)

    def callback_counter(self,msg):
        self.nbr_of_msgs += 1

if __name__ == '__main__':
    rospy.init_node('msgs_counter')
    count = Counter()
    rate = rospy.Rate(200)

    count.start_measuring()

    while rospy.Time.now() - count.start_time < count.interval:
        rate.sleep()
    
    print("Publishing frequency:", count.nbr_of_msgs/count.interval.to_sec())