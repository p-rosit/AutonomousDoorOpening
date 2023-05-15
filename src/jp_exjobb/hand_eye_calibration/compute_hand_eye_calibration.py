from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

import rospy

import numpy as np
import cv2 as cv
from scipy.spatial.transform import Rotation as rot

class ComputeHandEyeCalibration(SkillDescription):
    def createDescription(self):
        self.addParam(('start_hand_eye_calibration', 'started'), Element('skiros:Parameter'), ParamTypes.SharedInput)
        self.addParam(('save_hand_eye_calibration_poses', 'hand_poses'), Element('skiros:Parameter'), ParamTypes.SharedInput)
        self.addParam(('save_hand_eye_calibration_poses', 'marker_poses'), Element('skiros:Parameter'), ParamTypes.SharedInput)
        self.addParam('computed_transformation', Element('skiros:TransformationPose'), ParamTypes.SharedOutput)

class compute_hand_eye_calibration(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(ComputeHandEyeCalibration(), self.__class__.__name__)
    
    def onInit(self):
        self.preempted = False
        return True

    def preStart(self):
        self.preempted = False
        return True

    def onPreempt(self):
        self.preempted = True
        return self.fail('Computation of hand eye calibration was preempted.', -1)

    def run(self):
        hand_root = self.params['hand_poses'].value
        marker_root = self.params['marker_poses'].value

        hand_poses = self.get_poses_from(hand_root)
        marker_poses = self.get_poses_from(marker_root)

        hand_poses = self.transform_poses(hand_poses)
        marker_poses = self.transform_poses(marker_poses)

        poses = self.merge_poses(hand_poses, marker_poses)

        if self.preempted:
            return self.fail('Hand eye calibration preempted.', -1)

        hand_pos = np.array([pos for (pos, _), _ in poses])
        hand_rot = np.array([mat for (_, mat), _ in poses])
        marker_pos = np.array([pos for _, (pos, _) in poses])
        marker_rot = np.array([mat for _, (_, mat) in poses])

        camera_rot, camera_pos = cv.calibrateHandEye(hand_rot, hand_pos, marker_rot, marker_pos, method=cv.CALIB_HAND_EYE_TSAI)

        pose = self.params['computed_transformation'].value
        pose.setData(':Position', camera_pos.reshape(-1))
        pose.setData(':Orientation', rot.from_matrix(camera_rot).as_quat())

        self.setOutput('computed_transformation', pose)

        return self.success('Hand eye calibration computed.')
    
    def merge_poses(self, hand_poses, marker_poses):
        merged_poses = []

        for key, pose in hand_poses.items():
            if key not in marker_poses:
                rospy.logwarn('Hand pose is missing its buddy, are illegal things happening or is SkiROS lagging?')
                continue

            merged_poses.append((pose, marker_poses[key]))

        for key in marker_poses:
            if key not in hand_poses:
                rospy.logwarn('Marker pose is missing its buddy, are illegal things happening or is SkiROS lagging?')

        return merged_poses

    def transform_poses(self, poses):
        new_poses = dict()
        for key, (pos, quat) in poses.items():
            mat = rot.from_quat(quat).as_matrix()
            new_poses[key] = (pos, mat)
        return new_poses

    def get_poses_from(self, root):
        poses = dict()
        for root_relation in root.getRelations(subj='-1', pred='skiros:hasParam'):
            pose = self.wmi.get_element(root_relation['dst'])

            if pose.type != 'skiros:TransformationPose':
                rospy.logwarn('Found an unexpected element. Are you doing something illegal by writing poses to this parameter (%s) on your own?' % root.label)
                continue

            time_stamp = pose.getProperty('skiros:Value').value
            poses[time_stamp] = pose.getData(':Pose')

        return poses