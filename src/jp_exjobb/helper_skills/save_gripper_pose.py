from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes
from skiros2_skill.core.skill import SkillDescription

from scipy.spatial.transform import Rotation as rot
import numpy as np

class SaveGripperPose(SkillDescription):
    def createDescription(self):
        self.addParam('Gripper', Element('scalable:WsgGripper'), ParamTypes.Required)
        self.addParam('GripperPose', Element('skiros:TransformationPose'), ParamTypes.Required)

class save_gripper_pose(PrimitiveBase):
    """
    Summary:
        Saves the current gripper pose.

    Required Input:
        Gripper:        The gripper
        GripperPose:    Where to save the pose

    Behaviour:
        Reads the current pose of the gripper and updates the GripperPose
        transformation pose to contain the saved pose.
    """
    def createDescription(self):
        self.setDescription(SaveGripperPose(), self.__class__.__name__)
    
    def execute(self):
        gripper = self.params['Gripper'].value
        pose = self.params['GripperPose'].value

        msg = gripper.getData(':PoseStampedMsg')
        # Fix the rotation of the published gripper, because for some reason it is rotated...
        msg = fix_rotation(msg)

        pose.setData(':PoseStampedMsg',msg)
        self.wmi.update_element_properties(pose)
        return self.success('Gripper pose saved to %s.' % pose.label)

def fix_rotation(msg):
    sqrt_2 = 1 / np.sqrt(2)
    q = np.array([
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z,
        msg.pose.orientation.w
    ])
    q = (
        rot.from_quat(q) * rot.from_quat(np.array([0.0, sqrt_2, 0.0, sqrt_2]))
    ).as_quat()

    msg.pose.orientation.x = q[0]
    msg.pose.orientation.y = q[1]
    msg.pose.orientation.z = q[2]
    msg.pose.orientation.w = q[3]

    return msg