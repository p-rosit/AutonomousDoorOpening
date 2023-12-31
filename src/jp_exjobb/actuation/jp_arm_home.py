from skiros2_skill.core.skill import SkillBase, SkillDescription, Sequential
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

class JPArm(SkillDescription):
    def createDescription(self):
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Required)
        self.addParam('Home', Element('scalable:JointState'), ParamTypes.Inferred)
        self.addParam('JointController', Element('scalable:ControllerState'), ParamTypes.Inferred)

        self.addPreCondition(self.getPropCond('HomePose', 'skiros:Value', 'Home', '=', 'home_state', True))
        self.addPreCondition(self.getPropCond('Controller', 'skiros:Value', 'JointController', '=', 'joint_config', True))

class jp_arm_home(SkillBase):
    """
    Summary:
        Move arm to a set of home joint states.

    Required Input:
        Arm: The arm to control.

    Notes and Pitfalls:
        The arm is sent to the home state with the joint state controller.
        Self collisions are taken into account but collisions with the
        environment are not taken into account.
    """
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
