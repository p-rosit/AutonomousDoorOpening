from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.params import ParamTypes
import rospy
from std_msgs.msg import Empty, String

from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_std_skills.action_client_primitive import PrimitiveActionClient
import rospy
import actionlib
from jp_exjobb.msg import EmptyAction, EmptyGoal, EmptyResult, CameraCalibrationMsg
from std_msgs.msg import Int32, String

# from enum import Enum


# # Values from ros (rosmsg show actionlib/TestActionResult)
# class Status(Enum):
#     PENDING     = 0
#     ACTIVE      = 1
#     PREEMPTED   = 2
#     SUCCEEDED   = 3
#     ABORTED     = 4
#     REJECTED    = 5
#     PREEMPTING  = 6
#     RECALLING   = 7
#     RECALLED    = 8
#     LOST        = 9


# class ListenActionServer:
#     def __init__(self, topic):
#         print(topic)
#         # Set up action server on topic counter_as which takes uses the TestAction action messages
#         self._as = actionlib.SimpleActionServer("camera_calibration/action_server", EmptyAction, execute_cb=self.execute_callback, auto_start=False)
#         self._as.start()

#         self.topic = topic
#         self.pub = rospy.Publisher(self.topic, Empty, queue_size=1)
#         self.sub = rospy.Subscriber('/camera_calibration/response', String, callback=self.response_callback)

#         self.time_limit = 2
#         self.hz = 10

#     def response_callback(self, msg):
#         if msg.data == self.topic:
#             self.response = True

#     def execute_callback(self, goal):
#         r = rospy.Rate(self.hz)
#         count = 0
#         self.response = False
#         preempted = False

#         self.pub.publish(Empty())

#         while not self.response and count < self.time_limit * self.hz:
#             # If the action has been preempted break the loop
#             if self._as.is_preempt_requested():
#                 preempted = True
#                 break

#             r.sleep()
#             count += 1

#         if self.response:
#             # If the action succeeded set the corresponding state of the action server
#             self._as.set_succeeded(EmptyResult())
#         else:
#             if preempted:
#                 # If action preempted set the corresponding state of the action server
#                 self._as.set_preempted(EmptyResult())
#             else:
#                 # If action was aborted set the corresponding state of the action server
#                 self._as.set_aborted(EmptyResult())

# class camera_calibration_topic(PrimitiveActionClient):

#     def createDescription(self):
#         self.setDescription(CameraCalibration(), self.__class__.__name__)

#     def onStart(self):
#         # Runs each time the skill is started
#         self._as = ListenActionServer(self.topic)
#         return super().onStart()

#     def buildClient(self):
#         # Builds the action client which listens to the action server (the superclass saves
#         # this result to self.client in the background)
#         return actionlib.SimpleActionClient("camera_calibration/action_server", EmptyAction)
    
#     def buildGoal(self):
#         return EmptyGoal()
    
#     def restart(self, goal, text="Restarting action"):
#         # Restart server and the text is shown in SkiROS
#         self.client.send_goal(goal, done_cb=self._doneCb, feedback_cb=self._feedbackCb)
#         return self.step(text)
    
#     def onDone(self, status, msg):
#         # Called from execute, message is shown in SkiROS
#         if status == Status.SUCCEEDED.value:
#             return self.success(self.message)
#         else:
#             return self.fail("Camera calibration action server did not respond.", -1)

#     def execute(self):
#         if not self.res.empty():
#             # self.res is a result queue of size 1 (made by the superclass in the background)
#             # if the queue is nonempty we extract the message
#             result, status = self.res.get(False)

#             # Return the result which is shown in the SkiROS gui
#             return self.onDone(status, result)
        
#         # Message shown in SkiROS if there is nothing to report
#         return self.step("Publishing")

class CameraCalibration(SkillDescription):
    def createDescription(self):
        self.addParam('Camera', Element('skiros:'), ParamTypes.Required)

class camera_calibration_topic(PrimitiveBase):

    def createDescription(self):
        self.setDescription(CameraCalibration(), self.__class__.__name__)
    
    # self.topic and self.message need to be set in onInit by class extending this
    def onInit(self):
        self.running = False
        self.time_limit = 2
        self.hz = 10
        self.rate = rospy.Rate(self.hz)
        self.pub = rospy.Publisher(self.topic, Empty, queue_size=1)
        self.sub = rospy.Subscriber('/camera_calibration/response', CameraCalibrationMsg, callback=self.reponse_callback)
        return True
    
    def reponse_callback(self, msg):
        if self.running:
            if msg.data == self.topic:
                self.response = True
            else:
                rospy.logwarn('Incorrect response received, are several camera calibration skills running?')

    def onPreempt(self):
        return True
    
    def onStart(self):
        self.running = True
        self.response = False
        return True

    def execute(self):
        count = 0
        msg = CameraCalibrationMsg()
        self.pub.publish(msg)

        while not self.response and count < self.time_limit * self.hz:
            self.rate.sleep()
            count += 1

        if self.response:
            return self.success(self.message)
        else:
            return self.fail('Camera calibration action server did not respond.', -1)
    
    def onEnd(self):
        self.running = False
        return True

class start_camera_calibration(camera_calibration_topic):
    def onInit(self):
        self.topic = '/camera_calibration/start'
        self.message = 'Camera calibration action server started.'
        return super().onInit()

class take_picture(camera_calibration_topic):
    def onInit(self):
        self.topic = '/camera_calibration/take_picture'
        self.message = 'Picture taken.'
        return super().onInit()

class compute_intrinsic_camera_parameters(camera_calibration_topic):
    def onInit(self):
        self.topic = '/camera_calibration/compute_calibration'
        self.message = 'Intrinsic camera parameters computed.'
        return super().onInit()

class delete_previous_image(camera_calibration_topic):
    def onInit(self):
        self.topic = '/camera_calibration/delete'
        self.message = 'Deleted most recent image.'
        return super().onInit()
