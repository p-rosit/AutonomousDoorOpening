from skiros2_skill.core.skill import SkillBase, SkillDescription, Sequential, Loop
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

class JPEvaluateArm(SkillDescription):
    def createDescription(self):
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Required)
        self.addParam("Target", Element("skiros:TransformationPose"), ParamTypes.Required)
        self.addParam('Mode', Element('scalable:ControllerState'), ParamTypes.Required)
        self.addParam('Object', Element('skiros:Product'), ParamTypes.Required)
        self.addParam('Camera', Element('skiros:DepthCamera'), ParamTypes.Required)

class jp_actuation_evaluation(SkillBase):
    def createDescription(self):
        self.setDescription(JPEvaluateArm(), self.__class__.__name__)
    
    def expand(self, skill):
        skill.setProcessor(Sequential())
        skill(
            self.skill(Loop(end=20))(
                self.skill('JPMoveArm', 'jp_move_arm'),
                self.skill('DetectAndSave', 'detect_and_save', specify={'Object': self.params['Object'], 'Camera': self.params['Camera']}),
                self.skill('CalculateNavigation', 'calculate_mean'),
                self.skill('SwitchController', 'switch_controller', specify={'Controller': 'joint_config'}),
                self.skill('JPArm', 'jp_arm_home')
            ),
            self.skill('CalculateNavigation', 'evaluate_navigation')
        )
