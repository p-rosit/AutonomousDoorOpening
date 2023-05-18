from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes
from skiros2_skill.core.skill import SkillDescription

class SaveGripperPose(SkillDescription):
    def createDescription(self):
        self.addParam('Gripper', Element('scalable:WsgGripper'), ParamTypes.Required)
        self.addParam('GripperPose', Element('skiros:TransformationPose'), ParamTypes.Required)

class save_gripper_pose(PrimitiveBase):
    def createDescription(self):
        self.setDescription(SaveGripperPose(), self.__class__.__name__)
    
    def execute(self):
        gripper = self.params['Gripper'].value
        pose = self.params['GripperPose'].value

        pose.setData(':Pose', gripper.getData(':Pose'))
        self.wmi.update_element_properties(pose)
        return self.success('Gripper pose saved to %s.' % pose.label)
