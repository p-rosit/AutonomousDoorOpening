from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

import rospy
from std_msgs.msg import Empty, String, Bool
from geometry_msgs.msg import PoseStamped

class ComputeHandEyeCalibration(SkillDescription):
    def createDescription(self):
        self.addParam('Camera', Element('skiros:DepthCamera'), ParamTypes.Required)
        self.addParam('Hand', Element('sumo:Object'), ParamTypes.Required)
        self.addParam('Marker', Element('sumo:Object'), ParamTypes.Required)
        self.addParam('EE has camera', True, ParamTypes.Required)

class compute_hand_eye_calibration(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(ComputeHandEyeCalibration(), self.__class__.__name__)
    
    def onInit(self):
        self.running = False
        self.start_reply = False
        self.started = False
        self.preempted = False
        self.compute_pub = rospy.Subscriber('/hand_eye_calibration/camera_pose', PoseStamped, queue_size=1)
        self.hand_pub = rospy.Subscriber('/hand_eye_calibration/hand_pose', PoseStamped, callback=self.get_hand_pose)
        self.mark_pub = rospy.Subscriber('/hand_eye_calibration/marker_pose', PoseStamped, callback=self.get_marker_pose)

        self.start_pub = rospy.Publisher('/hand_eye_calibration/is_started', Empty, queue_size=1)
        self.start_sub = rospy.Subscriber('/hand_eye_calibration/is_started_reply', Bool, callback=self.start_callback)
        self.clear_sub = rospy.Subscriber('/hand_eye_calibration/clear', Empty, callback=self.clear)
        self.reply_pub = rospy.Publisher('/hand_eye_calibration/reply', String, queue_size=10)

        self.poses = []
        self.active_poses = [None, None]
        return True

    def start_callback(self, msg):
        if self.running:
            self.start_reply = True
            self.started = msg.data

    def get_hand_pose(self, msg):
        if self.active_poses[0] is None:
            self.active_poses[0] = msg.pose
        else:
            raise RuntimeError('Two hand poses were received but no marker pose.')
        
        if self.active_poses[1] is not None:
            self.poses.append(self.active_poses)
            self.active_poses = [None, None]
            print(self.poses)
    
        self.reply_pub.publish(String('hand'))

    def get_marker_pose(self, msg):
        if self.active_poses[1] is None:
            self.active_poses[1] = msg.pose
        else:
            raise RuntimeError('Two marker poses were received but no hand pose.')
        
        if self.active_poses[0] is not None:
            self.poses.append(self.active_poses)
            self.active_poses = [None, None]
            print(self.poses)
        
        self.reply_pub.publish(String('mark'))

    def clear(self, _):
        self.poses = []
        self.active_poses = [None, None]

    def preStart(self):
        self.running = True
        self.started = False
        self.reply_received = False
        self.preempted = False
        return True

    def onPreempt(self):
        self.preempted = True
        return self.fail('Computation of hand eye calibration was preempted.', -1)

    def run(self):
        self.start_pub.publish(Empty())
        
        ind = 0
        while ind < self.hz * self.time_limit and not self.start_reply:
            ind += 1
            self.rate.sleep()
        
        if not self.start_reply:
            return self.fail('Hand eye calibration server did not respond.')
        if not self.started:
            return self.fail('Hand eye calibration has not been started.')
        
        # compute parameters

        return self.success('Hand eye calibration computation timed out.')

    def onEnd(self):
        self.running = False
        return True