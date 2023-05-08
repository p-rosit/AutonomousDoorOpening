from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

class OperateHandle(SkillDescription):
    def createDescription(self):
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Required)
        self.addParam('EE', Element('scalable:Ur5EndEffector'), ParamTypes.Inferred)
        self.addParam('Gripper', Element('scalable:WsgGripper'), ParamTypes.Inferred)
        self.addParam('Pose', Element('skiros:TransformationPose'), ParamTypes.Inferred)
        self.addParam('Compliant', Element('scalable:ControllerState'), ParamTypes.Inferred)
        self.addPreCondition(self.getPropCond('CompliantController', 'skiros:Value', 'Compliant', '=', 'compliant', True))

        self.addParam('Handle', Element('scalable:DoorHandle'), ParamTypes.Required)

        self.addPreCondition(self.getRelationCond('ArmHasEE', 'skiros:hasA', 'Arm', 'EE', True))
        self.addPreCondition(self.getRelationCond('EEHasGripper', 'skiros:hasA', 'EE', 'Gripper', True))
        self.addPreCondition(self.getRelationCond('GripperHasPose', 'skiros:contain', 'Gripper', 'Pose', True))

class operate_handle(SkillBase):
    def createDescription(self):
        self.setDescription(OperateHandle(), self.__class__.__name__)

    def expand(self, skill):
        self.setProcessor(Sequential())
        gripper = self.params['Gripper'].value
        gstring = gripper.getProperty('scalable:GripperType').value

        if gstring == 'Push':
            skill(
                self.skill('SuccessSkill', 'success_skill', specify={
                    'msg': 'Door not opened, drive and pray.'
                })
            )
        elif gstring == 'Grasp':
            skill(
                self.skill('FailSkill', 'fail_skill', specfiy={
                    'msg': 'Not implemented.'
                })
            )
        else:
            skill(
                self.skill('FailSkill', 'fail_skill', specify={
                    'msg': 'Uknown gripper type.'
                })
            )
