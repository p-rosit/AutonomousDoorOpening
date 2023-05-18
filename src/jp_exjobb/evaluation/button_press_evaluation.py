from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential, Loop, InferInvalid
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

class EvaluateButtonPressing(SkillDescription):
    def createDescription(self):
        self.addParam('Heron', Element('cora:Robot'), ParamTypes.Required)
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Inferred)
        self.addPreCondition(self.getRelationCond('HeronHasArm', 'skiros:hasA', 'Heron', 'Arm', True))

        self.addParam('Button', Element('scalable:DoorButton'), ParamTypes.Required)
        self.addParam('ButtonLocation', Element('scalable:Location'), ParamTypes.Inferred)
        self.addPreCondition(self.getRelationCond('ButtonHasLocation', 'skiros:hasA', 'Button', 'ButtonLocation', True))

        self.addParam('StartLocation', Element('scalable:Location'), ParamTypes.Required)

        self.addParam('Loop Times', 10, ParamTypes.Required)

class evaluate_button_pressing(SkillBase):
    def createDescription(self):
        self.setDescription(EvaluateButtonPressing(), self.__class__.__name__)
    
    def expand(self, skill):
        self.setProcessor(Sequential())
        skill(
            self.skill(InferInvalid(Loop(self.params['Loop Times'].value)))(
                self.skill('JPDrive', 'jp_drive', specify={
                    'Heron': self.params['Heron'].value,
                    'TargetLocation': self.params['ButtonLocation'].value
                }),
                self.skill('JPDetectDOM', 'jp_detect_dom', specify={
                    'Arm': self.params['Arm'].value,
                    'Mechanism': self.params['Button'].value
                }),
                self.skill('ButtonPress', 'button_press', specify={
                    'Arm': self.params['Arm'].value,
                    'Button': self.params['Button'].value
                }),
                self.skill('JPArm', 'jp_arm_home', specify={
                    'Arm': self.params['Arm'].value
                }),
                self.skill('JPDrive', 'jp_drive', specify={
                    'Heron': self.params['Heron'].value,
                    'TargetLocation': self.params['StartLocation'].value
                })
            )
        )