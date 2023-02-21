import os

from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

import rospy
import tf2_ros
from std_msgs.msg import Float64MultiArray

import numpy as np
from scipy.spatial.transform import Rotation
from .aruco_evaluation import ArucoEvaluation
from .object_pose_skill import make_pose_stamped, unpack_pose_stamped, quat2rot, rot2quat

class ComputeMapTransformation(SkillDescription):
    def createDescription(self):
        pass

class compute_map_transformation(PrimitiveBase):
    def createDescription(self):
        self.setDescription(ComputeMapTransformation(), self.__class__.__name__)
    
    def onInit(self):
        self.map_pos = []
        self.map_quat = []
        self.gazebo_pos = []
        self.gazebo_quat = []
        self.t1_sub = rospy.Subscriber('/map_transformation/translation_map', Float64MultiArray, callback=lambda msg: self.callback(self.map_pos, msg))
        self.q1_sub = rospy.Subscriber('/map_tranformation/orientation_map', Float64MultiArray, callback=lambda msg: self.callback(self.map_quat, msg))
        self.t2_sub = rospy.Subscriber('/map_transformation/translation_gazebo', Float64MultiArray, callback=lambda msg: self.callback(self.gazebo_pos, msg))
        self.q2_sub = rospy.Subscriber('/map_tranformation/orientation_gazebo', Float64MultiArray, callback=lambda msg: self.callback(self.gazebo_quat, msg))
    
    def callback(self, data, msg):
        data.append(msg.data)

    def execute(self):
        print(self.map_pos)
        print(self.map_quat)
        print(self.gazebo_pos)
        print(self.gazebo_quat)
        map_to_gazebo_position = np.zeros(3)
        map_to_gazebo_orientation = np.ones(4)

        for map_position, map_orientation, gazebo_position, gazebo_orientation in zip(self.map_pos, self.map_quat, self.gazebo_pos, self.gazebo_quat):
            # Get rotation matrix of marker in map frame
            map_rotation_matrix = quat2rot(map_orientation)

            # Get rotation matrix of marker in Gazebo frame
            gazebo_rotation_matrix = quat2rot(gazebo_orientation)

            # Compute transformation between object parent frame and object such that the
            # set of transformations is consistent
            map_to_gazebo_rotation_matrix = map_rotation_matrix @ gazebo_rotation_matrix.T

            # Average over entire list
            map_to_gazebo_position += map_position - map_to_gazebo_rotation_matrix @ gazebo_position
            map_to_gazebo_orientation += rot2quat(map_to_gazebo_rotation_matrix)


        map_to_gazebo_position /= len(self.map_pos)
        map_to_gazebo_orientation /= np.linalg.norm(map_to_gazebo_orientation)

        print('t:', map_to_gazebo_position)
        print('q:', map_to_gazebo_orientation)

        self.ts = []
        self.qs = []

        return self.success('Computed transformation between map coordinate frame and gazebo coordinate frame.')

class sample_map_transformation(SkillBase):

    def createDescription(self):
        self.setDescription(ArucoEvaluation(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(Sequential())
        skill(
            self.skill('ArucoEstimation', 'aruco_marker', remap={'Object': 'ArUco marker'}),
            self.skill('ArucoEvaluation','save_coordinates', specify={'R': self.params['R'].values, 't': self.params['t'].values})
        )


class save_coordinates(PrimitiveBase):

    def createDescription(self):
        self.setDescription(ArucoEvaluation(), self.__class__.__name__)

    def onInit(self):
        self.buffer = tf2_ros.Buffer()  # type: any
        self.tf_listener = tf2_ros.TransformListener(self.buffer)
        self.t1_pub = rospy.Publisher('/map_transformation/translation_map', Float64MultiArray, queue_size=1)
        self.q1_pub = rospy.Publisher('/map_tranformation/orientation_map', Float64MultiArray, queue_size=1)
        self.t2_pub = rospy.Publisher('/map_transformation/translation_gazebo', Float64MultiArray, queue_size=1)
        self.q2_pub = rospy.Publisher('/map_tranformation/orientation_gazebo', Float64MultiArray, queue_size=1)
        self.msg = Float64MultiArray()
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
        # Extract position of object
        t_hat = np.array([aruco.getProperty('skiros:PositionX').value, 
                    aruco.getProperty('skiros:PositionY').value, 
                    aruco.getProperty('skiros:PositionZ').value])
        # Convert coordinate from object base frame to map
        print('hat', t_hat, quat_hat)
        estimated_pose = make_pose_stamped(aruco.getProperty('skiros:BaseFrameId').value, t_hat, quat_hat)
        estimated_map_pose = self.buffer.transform(estimated_pose, "map", rospy.Duration(1))
        t_hat, quat_hat = unpack_pose_stamped(estimated_map_pose)
        print('map_hat', t_hat, quat_hat)

        # Send estimated data
        self.msg.data = t_hat
        self.t1_pub.publish(self.msg)
        self.msg.data = quat_hat
        self.q1_pub.publish(self.msg)
        
        # True orientation of object as quaternion
        eul_ang = self.params['R'].values
        quat = Rotation.from_euler('xyz', eul_ang, degrees=False).as_quat()
        # True position of object
        t = self.params['t'].values

        print('real', t, quat)

        # Send Gazebo data
        self.msg.data = t
        self.t2_pub.publish(self.msg)
        self.msg.data = quat
        self.q2_pub.publish(self.msg)

        return self.success('Done')

    def onEnd(self):
        return True
