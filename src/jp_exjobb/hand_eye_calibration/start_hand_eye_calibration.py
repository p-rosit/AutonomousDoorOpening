from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

import rospy
from std_msgs.msg import Empty, String

class StartHandEyeCalibration(SkillDescription):
    def createDescription(self):
        pass

class start_hand_eye_calibration(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(StartHandEyeCalibration(), self.__class__.__name__)
    
    def onInit(self):
        self.hz = 10
        self.time_limit = 1.0
        self.rate = rospy.Rate(self.hz)

        self.running = False
        self.reply_received = False
        self.started = False
        self.pub = rospy.Publisher('/hand_eye_calibration/start', Empty, queue_size=1)
        self.sub = rospy.Subscriber('/hand_eye_calibration/reply', String, callback=self.reply_callback)
        return True

    def preStart(self):
        self.running = True
        self.reply_received = False
        self.started = False
        return True

    def reply_callback(self, msg):
        if self.running:
            self.reply_received = True
            if msg.data == 'started':
                self.started = True
            elif msg.data != 'start_error':
                raise ValueError('Received unexpected message: "%s".' % msg.data)

    def run(self):
        self.pub.publish(Empty())

        ind = 0
        while ind < self.hz * self.time_limit and not self.reply_received:
            ind += 1
            self.rate.sleep()
        
        if not self.reply_received:
            return self.fail('No reply received from the hand eye calibration server within %.1f seconds.' % self.time_limit, -1)
        
        if not self.started:
            return self.fail('Hand eye calibration has already been started.', -1)

        return self.success('Hand eye calibration started.')

    def onEnd(self):
        self.running = False
        return True