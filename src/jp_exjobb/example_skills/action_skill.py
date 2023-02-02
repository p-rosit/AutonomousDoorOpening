from skiros2_skill.core.skill import SkillDescription, ParamOptions, SkillBase, Sequential
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_std_skills.action_client_primitive import PrimitiveActionClient
import actionlib
from std_msgs.msg import Int32
# from actionlib.msg import TestAction, TestGoal, TestFeedback, TestResult
from jp_exjobb.msg import TestMsgAction, TestMsgGoal, TestMsgFeedback, TestMsgResult
import rospy

from enum import Enum

# Values from ros (rosmsg show actionlib/TestActionResult)
class Status(Enum):
    PENDING     = 0
    ACTIVE      = 1
    PREEMPTED   = 2
    SUCCEEDED   = 3
    ABORTED     = 4
    REJECTED    = 5
    PREEMPTING  = 6
    RECALLING   = 7
    RECALLED    = 8
    LOST        = 9


class ActionSkillServer:
    def __init__(self):
        # Set up action server on topic counter_as which takes uses the TestAction action messages
        self._as = actionlib.SimpleActionServer("counter_as_native", TestMsgAction, execute_cb=self.execute_callback, auto_start=False)
        self._as.start()
        rospy.loginfo('"counter_as": Started action server.')

        # Publisher which publishes numbers alongside the feedback
        self._pub = rospy.Publisher('/counter_as_native/counter', Int32, queue_size=1)

        # All instances of messages that will be needed
        self._num = Int32()
        self._feedback = TestMsgFeedback()
        self._result = TestMsgResult()

    def execute_callback(self, goal):
        r = rospy.Rate(1)
        success = True
        preempted = False

        rospy.loginfo('"counter_as_native: Started working on goal: %d.' % goal.goal)

        self._num.data = 0
        for ind in range(goal.goal):
            # Publish number, placeholder for some code that the action server should execute
            self._pub.publish(self._num)

            # If the action has been preempted break the loop
            if self._as.is_preempt_requested():
                rospy.loginfo('"counter_as_native": Preempting action on: %d.' % ind)
                success = False
                preempted = True
                break

            # Publish feedback
            rospy.loginfo('"counter_as_native": Publishing feedback on: %d.' % (ind - 1))
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
            rospy.loginfo('"counter_as_native": Goal succeeded at: %d.' % goal.goal)
            self._as.set_succeeded(self._result)
        else:
            self._result.result = goal.goal - ind
            if preempted:
                # If action preempted set the corresponding state of the action server
                rospy.loginfo('"counter_as_native": Goal preempted at: %d.' % (goal.goal - ind))
                self._as.set_preempted(self._result)
            else:
                # If action was aborted set the corresponding state of the action server
                rospy.loginfo('"counter_as_native": Goal aborted at: %d.' % (goal.goal - ind))
                self._as.set_aborted(self._result)


class ActionSkill(SkillDescription):
    def createDescription(self):
        self.addParam('Number', 0, ParamTypes.Required)


class action_skill(PrimitiveActionClient):

    def createDescription(self):
        self.setDescription(ActionSkill(), self.__class__.__name__)

    def onStart(self):
        # Runs each time the skill is started.
        self.retry_count = 0
        return super().onStart()

    def buildClient(self):
        # Builds the action client which listens to the action server (the superclass saves
        # this result to self.client in the background)
        return actionlib.SimpleActionClient("counter_as", TestMsgAction)
    
    def buildGoal(self):
        # Build the goal to be sent to the action server (the superclass does this in the background)
        goal = TestMsgGoal()
        goal.goal = self.params['Number'].value
        return goal
    
    def restart(self, goal, text="Restarting action"):
        # Restart server and the text is shown in SkiROS
        self.client.send_goal(goal, done_cb=self._doneCb, feedback_cb=self._feedbackCb)
        return self.step(text)

    def onFeedback(self, msg):
        # Feedback which is shown in SkiROS
        return self.step("Feedback: %d" % msg.feedback)
    
    def onDone(self, status, msg):
        # Called from execute, message is shown in SkiROS
        if status == Status.SUCCEEDED.value:
            return self.success("Goal succeeded")
        else:
            return self.fail("Goal failed", -1)

    def execute(self):
        if not self.fb.empty():
            # self.fb is a feedback queue of size 1 (made by the superclass in the background)
            # if the queue is nonempty we get the message and send it to the SkiROS gui.
            feedback = self.fb.get(False)
            return self.onFeedback(feedback)

        elif not self.res.empty():
            # self.res is a result queue of size 1 (made by the superclass in the background)
            # if the queue is nonempty we extract the message
            result, status = self.res.get(False)

            if status == Status.SUCCEEDED.value or self.retry_count >= 3:
                # If the goal succeeded or we have run out of tries we return the result which is
                # shown in the SkiROS gui
                return self.onDone(status, result)
            else:
                # Simple code for making a goal retry if it failed once.
                self.retry_count += 1
                rospy.loginfo('"counter_as": Server retrying goal %d.' % self.params['Number'].value)
                self.restart(self.buildGoal(), text="Restarting action.")
        
        # Message shown in SkiROS if there is nothing to report
        return self.step("running")


class action_skill_native(PrimitiveActionClient):

    def createDescription(self):
        self.setDescription(ActionSkill(), self.__class__.__name__)
    
    def onInit(self):
        # Only runs when SkiROS starts, therefore we can make and start the action server here.
        self._as = ActionSkillServer()
        return super().onInit()

    def onStart(self):
        # Runs each time the skill is started.
        self.retry_count = 0
        return super().onStart()

    def buildClient(self):
        # Builds the action client which listens to the action server (the superclass saves
        # this result to self.client in the background)
        return actionlib.SimpleActionClient("counter_as_native", TestMsgAction)
    
    def buildGoal(self):
        # Build the goal to be sent to the action server (the superclass does this in the background)
        goal = TestMsgGoal()
        goal.goal = self.params['Number'].value
        return goal
    
    def restart(self, goal, text="Restarting action"):
        # Restart server and the text is shown in SkiROS
        self.client.send_goal(goal, done_cb=self._doneCb, feedback_cb=self._feedbackCb)
        return self.step(text)

    def onFeedback(self, msg):
        # Feedback which is shown in SkiROS
        return self.step("Feedback: %d" % msg.feedback)
    
    def onDone(self, status, msg):
        # Called from execute, message is shown in SkiROS
        if status == Status.SUCCEEDED.value:
            return self.success("Goal succeeded")
        else:
            return self.fail("Goal failed", -1)

    def execute(self):
        if not self.fb.empty():
            # self.fb is a feedback queue of size 1 (made by the superclass in the background)
            # if the queue is nonempty we get the message and send it to the SkiROS gui.
            feedback = self.fb.get(False)
            return self.onFeedback(feedback)

        elif not self.res.empty():
            # self.res is a result queue of size 1 (made by the superclass in the background)
            # if the queue is nonempty we extract the message
            result, status = self.res.get(False)

            if status == Status.SUCCEEDED.value or self.retry_count >= 3:
                # If the goal succeeded or we have run out of tries we return the result which is
                # shown in the SkiROS gui
                return self.onDone(status, result)
            else:
                # Simple code for making a goal retry if it failed once.
                self.retry_count += 1
                rospy.loginfo('"counter_as_native": Server retrying goal %d.' % self.params['Number'].value)
                self.restart(self.buildGoal(), text="Restarting action.")
        
        # Message shown in SkiROS if there is nothing to report
        return self.step("running")