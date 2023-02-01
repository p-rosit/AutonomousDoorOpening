#! /usr/bin/env python

import actionlib
from std_msgs.msg import Int32
# from actionlib.msg import TestAction, TestGoal, TestFeedback, TestResult
from jp_exjobb.msg import TestMsgAction, TestMsgGoal, TestMsgFeedback, TestMsgResult
import rospy

class ActionSkillServer:
    def __init__(self):
        # Set up action server on topic counter_as which takes uses the TestAction action messages
        self._as = actionlib.SimpleActionServer("counter_as", TestMsgAction, execute_cb=self.execute_callback, auto_start=False)
        self._as.start()
        rospy.loginfo('"counter_as": Started action server.')

        # Publisher which publishes numbers alongside the feedback
        self._pub = rospy.Publisher('/counter_as/counter', Int32, queue_size=1)

        # All instances of messages that will be needed
        self._num = Int32()
        self._feedback = TestMsgFeedback()
        self._result = TestMsgResult()

    def execute_callback(self, goal):
        r = rospy.Rate(1)
        success = True
        preempted = False

        rospy.loginfo('"counter_as: Started working on goal: %d.' % goal.goal)

        self._num.data = 0
        for ind in range(goal.goal):
            # Publish number, placeholder for some code that the action server should execute
            self._pub.publish(self._num)

            # If the action has been preempted break the loop
            if self._as.is_preempt_requested():
                rospy.loginfo('"counter_as": Preempting action on: %d.' % ind)
                success = False
                preempted = True
                break

            # Publish feedback
            rospy.loginfo('"counter_as": Publishing feedback on: %d.' % (ind - 1))
            self._feedback.feedback = self._num.data - 1
            self._as.publish_feedback(self._feedback)

            # Contrived example of how a skill might fail
            if ind >= 3:
                success = False
                break

            # Sleep to avoid counting too fast
            r.sleep()
            self._num.data += 1
        
        if success:
            # If the action succeeded set the corresponding state of the action server
            self._result.result = goal.goal
            rospy.loginfo('"counter_as": Goal succeeded at: %d.' % goal.goal)
            self._as.set_succeeded(self._result)
        else:
            self._result.result = goal.goal - ind
            if preempted:
                # If action preempted set the corresponding state of the action server
                rospy.loginfo('"counter_as": Goal preempted at: %d.' % (goal.goal - ind))
                self._as.set_preempted(self._result)
            else:
                # If action was aborted set the corresponding state of the action server
                rospy.loginfo('"counter_as": Goal aborted at: %d.' % (goal.goal - ind))
                self._as.set_aborted(self._result)

if __name__ == '__main__':
    rospy.init_node('action_skill_server')
    asss = ActionSkillServer()
    rospy.spin()
