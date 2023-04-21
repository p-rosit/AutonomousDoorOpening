from skiros2_skill.core.skill import SkillBase, SkillDescription, Sequential, ParallelFs
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

class JPDetectAndMoveArm(SkillDescription):
    def createDescription(self):
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Required)
        self.addParam('Gripper', Element('scalable:WsgGripper'), ParamTypes.Inferred)
        self.addParam('Camera', Element('skiros:DepthCamera'), ParamTypes.Inferred)
        self.addPreCondition(self.getRelationCond('ArmHasGripper', 'skiros:hasA', 'Arm', 'Gripper', True))
        self.addPreCondition(self.getRelationCond('GripperContainsCamera', 'skiros:contain', 'Gripper', 'Camera', True))

        self.addParam('Object', Element('sumo:Object'), ParamTypes.Required)
        self.addParam('Mode', Element('scalable:ControllerState'), ParamTypes.Required)
        self.addParam('Target', Element('skiros:TransformationPose'), ParamTypes.Required)

        self.addParam('x', 0.04, ParamTypes.Required)
        self.addParam('y', 0.0, ParamTypes.Required)
        self.addParam('z', 0.0, ParamTypes.Required)

class jp_detect_and_move_arm(SkillBase):
    def createDescription(self):
        self.setDescription(JPDetectAndMoveArm(), self.__class__.__name__)
    
    def expand(self, skill):
        self.setProcessor(Sequential())

        skill(
            self.skill(ParallelFs())(
                self.skill('JPPoseEstimation', 'jp_pose_estimation', specify={
                    'Camera': self.params['Camera'].value,
                    'Object': self.params['Object'].value,
                    'Detection Time (s)': 1000.0
                }),
                self.skill('JPMoveArm', 'jp_move_arm', specify={
                    'Arm': self.params['Arm'].value,
                    'Target': self.params['Target'].value
                })
            )
        )