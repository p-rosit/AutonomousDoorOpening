from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes
from skiros2_skill.core.skill import SkillDescription

import rospy
from geometry_msgs.msg import PoseStamped
import tf2_ros
import numpy as np

rng = np.random.default_rng()

class JPGeneratePose(SkillDescription):
    def createDescription(self):
        self.addParam('Base', Element('sumo:Object'), ParamTypes.Required)
        self.addParam('Arm Base', Element('sumo:Object'), ParamTypes.Required)
        self.addParam('Prev', Element('sumo:Object'), ParamTypes.Required)
        self.addParam('Pose', Element('skiros:TransformationPose'), ParamTypes.Required)
        
        self.addParam('Radius', 1.0, ParamTypes.Required)
        self.addParam('Arm Radius', 1000.0, ParamTypes.Required)
        self.addParam('Min Angle', 0.3, ParamTypes.Required)

class jp_generate_pose(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(JPGeneratePose(), self.__class__.__name__)
    
    def onInit(self):
        self.buffer = tf2_ros.Buffer()  # type: any
        self.tf_listener = tf2_ros.TransformListener(self.buffer)
        return True

    def run(self):
        self.base = self.params['Base'].value
        self.arm_base = self.params['Arm Base'].value
        self.prev = self.params['Prev'].value
        self.pose = self.params['Pose'].value

        self.rad = self.params['Radius'].value
        self.arm_rad = self.params['Arm Radius'].value
        self.ang = self.params['Min Angle'].value

        if not self.base.hasData(':Pose'):
            return self.fail('Base does not have a pose.', -1)
        
        self.has_prev = self.prev.label != '' and self.prev.id != ''

        if self.has_prev and not self.prev.hasData(':Pose'):
            return self.fail('Previous point does not have a pose.', -1)
        
        self.has_arm_base = self.arm_base.label != '' and self.arm_base.id != ''
        if not self.has_arm_base:
            self.arm_rad = np.inf

        if self.has_arm_base and not self.arm_base.hasData(':Pose'):
            return self.fail('Arm does not have a pose.')

        self.ang = self.ang if 0 <= self.ang <= np.pi else np.pi
        self.pos = self.generate_position()
        self.quat = np.array([0, 0, 0, 1])

        self.convert_pose()
        self.pose.setData(':Position', self.pos)
        self.wmi.update_element_properties(self.pose)

        return self.success('Yes')

    def generate_position(self):
        if self.has_arm_base:
            arm_pose = self.arm_base.getData(':PoseStampedMsg')
            arm_pose = self.buffer.transform(arm_pose, self.base.getProperty('skiros:FrameId').value, rospy.Duration(1))
            arm_pos = np.array([
                arm_pose.pose.position.x,
                arm_pose.pose.position.y,
                arm_pose.pose.position.z
            ])
        else:
            arm_pos = np.zeros(3)
        
        pos = np.full(3, np.inf)
        angle = np.arctan2(np.linalg.norm(pos[:2]), pos[2])
        arm_mag = np.square(pos - arm_pos).sum()

        while angle > self.ang or arm_mag > self.arm_rad:
            pos = rng.normal(size=3)
            pos *= self.rad / np.linalg.norm(pos)

            angle = np.arctan2(np.linalg.norm(pos[:2]), pos[2])
            arm_mag = np.square(pos - arm_pos).sum()

        return pos

    def convert_pose(self):
        pose = PoseStamped()
        pose.header.frame_id = self.base.getProperty('skiros:FrameId').value
        pose.header.seq = rospy.Time(0)

        pose.position.x = self.pos[0]
        pose.position.y = self.pos[1]
        pose.position.z = self.pos[2]

        pose.orientation.x = self.quat[0]
        pose.orientation.y = self.quat[1]
        pose.orientation.z = self.quat[2]
        pose.orientation.w = self.quat[3]

        pose = self.buffer.transform(pose, self.pose.getProperty('skiros:BaseFrameId').value, rospy.Duration(1))
        
        self.pos[0] = pose.position.x
        self.pos[1] = pose.position.y
        self.pos[2] = pose.position.z

        self.quat[0] = pose.orientation.x
        self.quat[1] = pose.orientation.y
        self.quat[2] = pose.orientation.z
        self.quat[3] = pose.orientation.w