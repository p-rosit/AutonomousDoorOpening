from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential, Loop, InferInvalid
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

class EvaluateDoorPass(SkillDescription):
    def createDescription(self):
        self.addParam('Heron', Element('cora:Robot'), ParamTypes.Required)
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Inferred)
        self.addPreCondition(self.getRelationCond('HeronHasArm', 'skiros:hasA', 'Heron', 'Arm', True))

        self.addParam('Button1', Element('scalable:DoorButton'), ParamTypes.Required)
        self.addParam('Button1Location', Element('scalable:Location'), ParamTypes.Inferred)
        self.addPreCondition(self.getRelationCond('Button1HasLocation', 'skiros:hasA', 'Button1', 'Button1Location', True))
        self.addParam('Button2', Element('scalable:DoorButton'), ParamTypes.Required)
        self.addParam('Button2Location', Element('scalable:Location'), ParamTypes.Inferred)
        self.addPreCondition(self.getRelationCond('Button2HasLocation', 'skiros:hasA', 'Button2', 'Button2Location', True))

        self.addParam('DoorPoint1', Element('scalable:Location'), ParamTypes.Required)
        self.addParam('DoorPoint2', Element('scalable:Location'), ParamTypes.Required)

        self.addParam('Loop Times', 30, ParamTypes.Required)

class evaluate_door_pass(SkillBase):
    def createDescription(self):
        self.setDescription(EvaluateDoorPass(), self.__class__.__name__)
    
    def expand(self, skill):
        self.setProcessor(Sequential())
        skill(
            self.skill(InferInvalid(Loop(self.params['Loop Times'].value)))(
                self.skill('JPDrive', 'jp_drive', specify={
                    'Heron': self.params['Heron'].value,
                    'TargetLocation': self.params['DoorPoint1'].value
                }),
                self.skill('JPDrive', 'jp_drive', specify={
                    'Heron': self.params['Heron'].value,
                    'TargetLocation': self.params['Button1Location'].value
                }),
                self.skill('JPDetectDOM', 'jp_detect_dom', specify={
                    'Arm': self.params['Arm'].value,
                    'Mechanism': self.params['Button1'].value
                }),
                self.skill('ButtonPress', 'button_press', specify={
                    'Arm': self.params['Arm'].value,
                    'Button': self.params['Button1'].value
                }),
                self.skill('JPArm', 'jp_arm_home', specify={
                    'Arm': self.params['Arm'].value
                }),
                self.skill('JPDrive', 'jp_drive', specify={
                    'Heron': self.params['Heron'].value,
                    'TargetLocation': self.params['DoorPoint2'].value
                }),
                self.skill('JPDrive', 'jp_drive', specify={
                    'Heron': self.params['Heron'].value,
                    'TargetLocation': self.params['Button2Location'].value
                }),
                self.skill('JPDetectDOM', 'jp_detect_dom', specify={
                    'Arm': self.params['Arm'].value,
                    'Mechanism': self.params['Button2'].value
                }),
                self.skill('ButtonPress', 'button_press', specify={
                    'Arm': self.params['Arm'].value,
                    'Button': self.params['Button2'].value
                }),
                self.skill('JPArm', 'jp_arm_home', specify={
                    'Arm': self.params['Arm'].value
                })
            )
        )