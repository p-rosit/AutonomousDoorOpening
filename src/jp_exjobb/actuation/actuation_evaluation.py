from skiros2_skill.core.skill import SkillBase, Sequential, Loop
from jp_exjobb.actuation.actuation_skill import JPMoveArm

class jp_compliant_evaluation(SkillBase):
    def createDescription(self):
        self.setDescription(JPMoveArm(), self.__class__.__name__)
    
    def expand(self, skill):
        skill.setProcessor(Sequential())
        skill(
            self.skill(Loop(end=20))(
                self.skill('JPMoveArm', 'jp_compliant_move_arm'),
                self.skill('DetectAndSave', 'detect_and_save', remap={'Object': 'Object', 'Camera': 'Camera'}),
                self.skill('CalculateNavigation', 'calculate_mean'),
                self.skill('SwitchController', 'switch_controller', specify={'Controller': 'joint_config'}),
                self.skill('JPArm', 'jp_arm_home')
            ),
            self.skill('CalculateNavigation', 'evaluate_navigation')
        )

