from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

import rospy
from std_msgs.msg import Empty, String, Bool
from geometry_msgs.msg import PoseStamped

class SaveHandEyeCalibration(SkillDescription):
    def createDescription(self):
        self.addParam('Camera', Element('skiros:DepthCamera'), ParamTypes.Required)
        self.addParam('Hand', Element('sumo:Object'), ParamTypes.Required)
        self.addParam('Marker', Element('sumo:Object'), ParamTypes.Required)
        self.addParam('EE has camera', True, ParamTypes.Required)

class save_hand_eye_calibration(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(SaveHandEyeCalibration(), self.__class__.__name__)
    
    def onInit(self):
        self.hz = 10
        self.time_limit = 10
        self.rate = rospy.Rate(self.hz)

        self.started = False
        self.computed = False
        self.preempted = False
        self.start_sub = rospy.Subscriber('/hand_eye_calibration/start', Empty, callback=self.start_callback)
        self.started_sub = rospy.Subscriber('/hand_eye_calibration/is_started', Empty, callback=self.is_started_callback)
        self.started_pub = rospy.Publisher('/hand_eye_calibration/is_started_reply', Bool, queue_size=1)
        self.reply_pub = rospy.Publisher('/hand_eye_calibration/reply', String, queue_size=10)
        return True

    def preStart(self):
        self.started = False
        self.computed = False
        self.preempted = False
        return True

    def start_callback(self, _):
        if not self.started:
            self.started = True
            self.reply_pub.publish(String('started'))
        else:
            self.reply_pub.publish(String('start_error'))

    def is_started_callback(self, _):
        self.started_pub.publish(Bool(self.started))

    def onPreempt(self):
        self.preempted = True
        return self.fail('Hand eye calibration parameters were not saved.', -1)

    def run(self):
        if not self.started:
            return self.fail('Hand eye calibration has not been started.', -1)
        self.started = False

        ind = 0
        while ind < self.hz * self.time_limit and not self.computed and not self.preempted:
            ind += 1
            self.rate.sleep()

        if self.computed:
            return self.success('Hand eye calibration parameters were computed.')

        return self.success('Hand eye calibration computation timed out.')