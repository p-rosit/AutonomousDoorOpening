import os

from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

import rospy
from std_msgs.msg import Int32
import tf2_ros

import numpy as np
from scipy.spatial.transform import Rotation
from .object_pose_skill import make_pose_stamped, unpack_pose_stamped

class ArucoEvaluation(SkillDescription):
    def createDescription(self):
        self.addParam('ArUco marker', Element('skiros:Product'), ParamTypes.Required)
        self.addParam('R', [0.0, 0.0, 0.0], ParamTypes.Required)
        self.addParam('t', [0.0, 0.0, 0.0], ParamTypes.Required)

class aruco_evaluation(SkillBase):

    def createDescription(self):
        self.setDescription(ArucoEvaluation(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(Sequential())
        skill(
            self.skill('ArucoEstimation', 'aruco_marker', remap={'Object': 'ArUco marker'}),
            self.skill('ArucoEvaluation','coordinate_comparison')
        )


class coordinate_comparison(PrimitiveBase):

    def createDescription(self):
        self.setDescription(ArucoEvaluation(), self.__class__.__name__)

    def onInit(self):
        self.buffer = tf2_ros.Buffer()  # type: any
        self.tf_listener = tf2_ros.TransformListener(self.buffer)
        return True

    def onPreempt(self):
        return True

    def onStart(self):
        return True

    def execute(self):
        aruco = self.params['ArUco marker'].value
        # Extract orientation of object
        quat_hat = np.array([aruco.getProperty('skiros:OrientationX').value, 
                    aruco.getProperty('skiros:OrientationY').value, 
                    aruco.getProperty('skiros:OrientationZ').value, 
                    aruco.getProperty('skiros:OrientationW').value])
        quat_hat = Rotation.from_quat(quat_hat)

        # Extract position of object
        t_hat = np.array([aruco.getProperty('skiros:PositionX').value, 
                    aruco.getProperty('skiros:PositionY').value, 
                    aruco.getProperty('skiros:PositionZ').value])
        
        # True orientation of object
        eul_ang = self.params['R'].values
        quat = Rotation.from_euler('xyz', eul_ang, degrees=False).as_quat()
        # True position of object
        t = self.params['t'].values + np.array([28.1, 7.4, 0.0])
        print(t)

        # tranfering the measured pose from map to workspace
        object_parent_frame = aruco.getProperty('skiros:BaseFrameId').value
        true_pose = make_pose_stamped('map', t, quat)
        true_pose = self.buffer.transform(true_pose, object_parent_frame, rospy.Duration(1))
        t, quat = unpack_pose_stamped(true_pose)
        quat = Rotation.from_quat(quat)
        # Compute difference in orientation
        diff_quat = (quat*quat_hat.inv()).as_quat()
        if diff_quat[3] < 0:
            # Ensure that the quaternion is on the upper half-sphere
            diff_quat = -diff_quat

        # Compute the norm of the difference between the estimate pose and the true pose
        delta_quat = np.linalg.norm(diff_quat - np.array([0, 0, 0, 1]))
        delta_t = np.linalg.norm(t - t_hat)

        print('Delta quaternion: ', delta_quat)
        print('Delta translation:', delta_t)

        path = os.path.expanduser('~')
        with open(os.path.join(path, 'evaluation'), 'a', encoding='utf-8') as f:
            f.write('dq: %f\n' % delta_quat)
            f.write('dt: %f\n' % delta_t)
            f.write('---\n')

        return self.success('Done')

    def onEnd(self):
        return True
