from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

import rospy
from std_msgs.msg import Empty, Bool, String
from geometry_msgs.msg import PoseStamped

class JPSaveHandEyeCalibData(SkillDescription):
    def createDescription(self):
        self.addParam('Hand', Element('sumo:Object'), ParamTypes.Required)
        self.addParam('Marker', Element('sumo:Object'), ParamTypes.Required)

class jp_save_hand_eye_calib_data(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(JPSaveHandEyeCalibData(), self.__class__.__name__)

    def onInit(self):
        self.hz = 10
        self.time_limit = 1.0
        self.rate = rospy.Rate(self.hz)

        self.running = False

        self.start_reply = False
        self.started = False
        self.is_started_pub = rospy.Publisher('/hand_eye_calibration/is_started', Empty, queue_size=1)
        self.is_started_sub = rospy.Subscriber('/hand_eye_calibration/is_started_reply', Bool, callback=self.is_started_callback)

        self.reply_received = False
        self.hand_reply = False
        self.mark_reply = False
        self.hand_pub = rospy.Publisher('/hand_eye_calibration/hand_pose', PoseStamped, queue_size=1)
        self.mark_pub = rospy.Publisher('/hand_eye_calibration/marker_pose', PoseStamped, queue_size=1)

        self.reply_sub = rospy.Subscriber('/hand_eye_calibration/reply', String, callback=self.reply_callback)
        return True

    def preStart(self):
        hand = self.params['Hand'].value
        marker = self.params['Marker'].value

        if not hand.hasData(':Pose'):
            return self.startError('Hand does not have a pose.', -1)
    
        if not marker.hasData(':Pose'):
            return self.startError('Marker does not have a pose', -1)

        self.running = True
        self.reply_received = False
        self.hand_reply = False
        self.mark_reply = False
        return True

    def reply_callback(self, msg):
        if self.running:
            print(msg.data)
            if msg.data == 'hand':
                self.hand_reply = True
            elif msg.data == 'mark':
                self.mark_reply = True
            else:
                raise ValueError('Received unexpected message: "%s".' % msg.data)

    def is_started_callback(self, msg):
        if self.running:
            self.start_reply = True
            self.started = msg.data

    def run(self):
        self.is_started_pub.publish(Empty())
        
        ind = 0
        while ind < self.hz * self.time_limit and not self.start_reply:
            ind += 1
            self.rate.sleep()
        
        if not self.start_reply:
            return self.fail('Hand eye calibration server did not reply.', -1)
        if not self.start:
            return self.fail('Hand eye calibration has not been started', -1)
        
        hand = self.params['Hand'].value
        marker = self.params['Marker'].value

        hand_pose = hand.getData(':PoseStampedMsg')
        marker_pose = marker.getData(':PoseStampedMsg')

        self.hand_pub.publish(hand_pose)
        self.mark_pub.publish(marker_pose)

        ind = 0
        while ind < self.hz * self.time_limit and (not self.mark_reply or not self.hand_reply):
            ind += 1
            self.rate.sleep()

        if not self.mark_reply or not self.hand_reply:
            return self.fail('Poses were published but not received within %.1f seconds.' % self.time_limit, -1)

        return self.success('Collected and published poses.')

    def onEnd(self):
        self.running = False
        return True
