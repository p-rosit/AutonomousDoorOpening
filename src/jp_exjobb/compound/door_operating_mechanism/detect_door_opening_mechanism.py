from skiros2_skill.core.skill import SkillBase, SkillDescription, Sequential
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

class JPDetectDOM(SkillDescription):
    def createDescription(self):
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Required)
        self.addParam('EE', Element('scalable:Ur5EndEffector'), ParamTypes.Inferred)
        self.addParam('Gripper', Element('scalable:WsgGripper'), ParamTypes.Inferred)
        self.addParam('Camera', Element('skiros:DepthCamera'), ParamTypes.Inferred)
        self.addPreCondition(self.getRelationCond('ArmHasEE', 'skiros:hasA', 'Arm', 'EE', True))
        self.addPreCondition(self.getRelationCond('EEHasGripper', 'skiros:hasA', 'EE', 'Gripper', True))
        self.addPreCondition(self.getRelationCond('EEhasCamera', 'skiros:hasA', 'EE', 'Camera', True))

        self.addParam('DOM', Element('scalable:DoorOperatingMechanism'), ParamTypes.Required)
        self.addParam('JointState', Element('scalable:ControllerState'), ParamTypes.Inferred)
        self.addParam('Target', Element('scalable:JointState'), ParamTypes.Inferred)

        self.addPreCondition(self.getRelationCond('DOMHasLookout', 'skiros:hasA', 'DOM', 'Target', True))
        self.addPreCondition(self.getPropCond('JointStateController', 'skiros:Value', 'JointState', '=', 'joint_config', True))


class jp_detect_dom(SkillBase):
    def createDescription(self):
        self.setDescription(JPDetectDOM(), self.__class__.__name__)
    
    def expand(self, skill):
        self.setProcessor(Sequential())

        skill(
            self.skill('JPMoveArm', 'jp_move_arm', specify={
                'Arm': self.params['Arm'].value,
                'Target': self.params['Target'].value,
                'Mode': self.params['JointState'].value
            }),
            self.skill('JPPoseEstimation', 'jp_pose_estimation', specify={
                'Camera': self.params['Camera'].value,
                'Object': self.params['DOM'].value,
                'Detection Time (s)': 0.4
            })
        )
