import sys
import threading

from skiros2_skill.core.skill import SkillBase, SkillDescription, Sequential
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
# from skiros2_common.core.primitive import PrimitiveBase
# import moveit_commander

# import rospy

class JPArm(SkillDescription):
    def createDescription(self):
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Required)
        self.addParam('Home', Element('scalable:JointState'), ParamTypes.Inferred)
        self.addParam('JointController', Element('scalable:ControllerState'), ParamTypes.Inferred)

        self.addPreCondition(self.getPropCond('HomePose', 'skiros:Value', 'Home', '=', 'home_state', True))
        self.addPreCondition(self.getPropCond('Controller', 'skiros:Value', 'JointController', '=', 'joint_config', True))

class jp_arm_home(SkillBase):
    def createDescription(self):
        self.setDescription(JPArm(), self.__class__.__name__)
    
    def expand(self, skill):
        skill.setProcessor(Sequential())
        skill(
            self.skill('JPMoveArm', 'jp_move_arm', specify={
                    'Target': self.params['Home'].value,
                    'Mode': self.params['JointController'].value
                }
            )
        )
