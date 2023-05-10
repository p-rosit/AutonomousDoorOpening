from skiros2_skill.core.skill import SkillBase, SkillDescription, Sequential, ParallelFs
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

class MoveArmOnSphere(SkillDescription):
    def createDescription(self):
        self.addParam('Base', Element('sumo:Object'), ParamTypes.Required)

        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Required)
        self.addParam('EE', Element('scalable:Ur5EndEffector'), ParamTypes.Inferred)
        self.addParam('Gripper', Element('scalable:WsgGripper'), ParamTypes.Inferred)
        self.addParam('Pose', Element('skiros:TransformationPose'), ParamTypes.Inferred)

        self.addPreCondition(self.getRelationCond('ArmHasEE', 'skiros:hasA', 'Arm', 'EE', True))
        self.addPreCondition(self.getRelationCond('EEHasGripper', 'skiros:hasA', 'EE', 'Gripper', True))
        self.addPreCondition(self.getRelationCond('GripperHasPose', 'skiros:contain', 'Gripper', 'Pose', True))

        self.addParam('Direction Up', [0.0, 0.0, -1.0], ParamTypes.Optional)
        self.addParam('Direction Frame', '', ParamTypes.Optional)

        self.addParam('Radius', 0.7, ParamTypes.Required)
        self.addParam('Origin Max Radius', 0.8, ParamTypes.Optional)
        self.addParam('Origin Min Radius', 0.5, ParamTypes.Optional)
        self.addParam('Max Angle (deg)', 50.0, ParamTypes.Required)
        self.addParam('Angle Interval (deg)', 10.0, ParamTypes.Optional)
        self.addParam('Goal Wait (s)', 20.0, ParamTypes.Optional)

        self.addParam('Compliant', Element('scalable:ControllerState'), ParamTypes.Inferred)
        self.addPreCondition(self.getPropCond('CompliantController', 'skiros:Value', 'Compliant', '=', 'compliant', True))

class move_arm_on_sphere(SkillBase):
    def createDescription(self):
        self.setDescription(MoveArmOnSphere(), self.__class__.__name__)
    
    def expand(self, skill):
        self.setProcessor(Sequential())
        skill(
            self.skill('JPGeneratePose', 'jp_generate_pose', specify={
                'Base': self.params['Base'].value,
                'Origin': self.params['Arm'].value,
                'Prev': self.params['Gripper'].value,
                'Pose': self.params['Pose'].value,

                'Direction Up': self.params['Direction Up'].values,
                'Direction Frame': self.params['Direction Frame'].value,

                'Radius': self.params['Radius'].value,
                'Origin Max Radius': self.params['Origin Max Radius'].value,
                'Origin Min Radius': self.params['Origin Min Radius'].value,
                'Max Angle (deg)': self.params['Max Angle (deg)'].value,
                'Angle Interval (deg)': self.params['Angle Interval (deg)'].value
            }),
            self.skill(ParallelFs())(
                self.skill('TimerSkill', 'timer_skill', specify={
                    'Max Time (s)': self.params['Goal Wait (s)'].value
                }),
                self.skill('JPMoveArm', 'jp_move_arm', specify={
                    'Target': self.params['Pose'].value,
                    'Mode': self.params['Compliant'].value
                })
            )
        )
