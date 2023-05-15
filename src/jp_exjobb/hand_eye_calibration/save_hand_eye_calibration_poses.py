from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

import rospy
import tf2_ros

class SaveHandEyeCalibrationPoses(SkillDescription):
    def createDescription(self):
        self.addParam(('start_hand_eye_calibration', 'started'), Element('skiros:Parameter'), ParamTypes.SharedInput)
        self.addParam(('start_hand_eye_calibration', 'camera'), Element('skiros:DepthCamera'), ParamTypes.LinkedInput)
        self.addParam(('start_hand_eye_calibration', 'hand'), Element('sumo:Object'), ParamTypes.LinkedInput)
        self.addParam(('start_hand_eye_calibration', 'marker'), Element('sumo:Object'), ParamTypes.LinkedInput)

        self.addParam('View Frame', Element('skiros:TransformationPose'), ParamTypes.Inferred)
        self.addPreCondition(self.getRelationCond('CameraHasViewFrame', 'skiros:hasA', 'camera', 'View Frame', True))

        self.addParam('hand_poses', Element('skiros:Parameter'), ParamTypes.SharedOutput)
        self.addParam('marker_poses', Element('skiros:Parameter'), ParamTypes.SharedOutput)

class save_hand_eye_calibration_poses(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(SaveHandEyeCalibrationPoses(), self.__class__.__name__)

    def onInit(self):
        self.buffer = tf2_ros.Buffer()  # type: any
        self.tf_listener = tf2_ros.TransformListener(self.buffer)
        return True

    def preStart(self):
        hand = self.params['hand'].value
        marker = self.params['marker'].value

        if not hand.hasData(':Pose'):
            return self.startError('Hand does not have a pose.', -1)
    
        if not marker.hasData(':Pose'):
            return self.startError('Marker does not have a pose', -1)

        return True

    def run(self):
        start = self.params['started'].value
        camera = self.params['camera'].value
        hand_param = self.params['hand'].value
        marker_param = self.params['marker'].value

        if not start.hasProperty('skiros:Value', not_none=True):
            return self.fail('Start object does not have the correct parameter.')
        
        started = start.getProperty('skiros:Value').value
        if not started:
            return self.fail('Hand eye calibration has not been started.')

        self.status = 'Saving poses.'
        hand_poses = self.getOutput('hand_poses')
        marker_poses = self.getOutput('marker_poses')
        if hand_poses is None:
            hand_poses = self.params['hand_poses'].value
            hand_poses = self.setOutput('hand_poses', hand_poses)
        if marker_poses is None:
            marker_poses = self.params['marker_poses'].value
            marker_poses = self.setOutput('marker_poses', marker_poses)

        hand_pose = hand_param.getData(':PoseStampedMsg')
        marker_pose = marker_param.getData(':PoseStampedMsg')

        camera_frame = camera.getProperty('skiros:FrameId').value
        hand_pose = self.buffer.transform(hand_pose, 'map', rospy.Duration(1))
        marker_pose = self.buffer.transform(marker_pose, camera_frame, rospy.Duration(1))

        time_stamp = rospy.Time.now().to_sec()
        hand = Element('skiros:Parameter')
        hand = setPose(hand, hand_pose)
        hand.setProperty('skiros:Value', time_stamp)

        marker = Element('skiros:Parameter')
        marker = setPose(marker, marker_pose)
        marker.setProperty('skiros:Value', time_stamp)
        
        hand.addRelation(subj=hand_poses.id, predicate='skiros:hasParam', obj='-1')
        marker.addRelation(subj=marker_poses.id, predicate='skiros:hasParam', obj='-1')

        self.wmi.add_element(hand)
        self.wmi.add_element(marker)

        return self.success('Saved poses.')

def setPose(element, pose):
    element.setProperty('skiros:PositionX', pose.pose.position.x)
    element.setProperty('skiros:PositionY', pose.pose.position.y)
    element.setProperty('skiros:PositionZ', pose.pose.position.z)

    if pose.pose.orientation.w < 0:
        pose.pose.orientation.x = -pose.pose.orientation.x
        pose.pose.orientation.y = -pose.pose.orientation.y
        pose.pose.orientation.z = -pose.pose.orientation.z
        pose.pose.orientation.w = -pose.pose.orientation.w
    element.setProperty('skiros:OrientationX', pose.pose.orientation.x)
    element.setProperty('skiros:OrientationY', pose.pose.orientation.y)
    element.setProperty('skiros:OrientationZ', pose.pose.orientation.z)
    element.setProperty('skiros:OrientationW', pose.pose.orientation.w)

    return element