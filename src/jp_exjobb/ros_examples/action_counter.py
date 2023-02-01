#! /usr/bin/env python

import rospy
import actionlib
from std_msgs.msg import Int32, EmptyAction

class CounterAction:
    
    def __init__(self):
        self._as = actionlib.SimpleActionServer("counter_as", EmptyAction, self.goal_callback, False)
        self._as.start()
        self._pub = rospy.Publisher("/counting", Int32, queue_size=1)

    def goal_callback(self, goal):
        r = rospy.Rate(1)
        num = Int32()
        num.data = 0

        rospy.loginfo("Starting count")

        while not self._as.is_preempt_requested():
            self._pub.publish(num)
            num.data += 1


            r.sleep()


rospy.init_node('counter_action_client')


