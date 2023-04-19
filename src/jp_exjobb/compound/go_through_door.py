from skiros2_skill.core.skill import SkillBase, SkillDescription, Sequential
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes
# nörd

class GoThroughDoor(SkillDescription):
    def createDescription(self):
        self.addParam('Heron', Element('cora:Robot'), ParamTypes.Required)
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Inferred)
        self.addParam('Gripper', Element('scalable:WsgGripper'), ParamTypes.Inferred)
        self.addParam('Camera', Element('skiros:DepthCamera'), ParamTypes.Inferred)
        self.addPreCondition(self.getRelationCond('HeronHasArm', 'skiros:hasA', 'Heron', 'Arm', True))
        self.addPreCondition(self.getRelationCond('ArmHasGripper', 'skiros:hasA', 'Arm', 'Gripper', True))
        self.addPreCondition(self.getRelationCond('GripperContainsCamera', 'skiros:contain', 'Gripper', 'Camera', True))

        self.addParam('Button', Element('sumo:Object'), ParamTypes.Required)
        self.addParam('ButtonLookout', Element('scalable:JointState'), ParamTypes.Inferred)
        self.addPreCondition(self.getRelationCond('ButtonHasLookout', 'skiros:hasA', 'Button', 'ButtonLookout', True))

        self.addParam('TargetLocation', Element('scalable:Workstation'), ParamTypes.Required)
        self.addParam('ArmHome', Element('scalable:JointState'), ParamTypes.Inferred)
        self.addParam('JointState', Element('scalable:ControllerState'), ParamTypes.Inferred)
        self.addParam('Compliant', Element('scalable:ControllerState'), ParamTypes.Inferred)

        self.addPreCondition(self.getHasPropCond('HomePosition', 'rdfs:label', 'ArmHome', '=', 'home_state', True))
        self.addPreCondition(self.getHasPropCond('CompliantController', 'rdfs:label', 'Compliant', '=', 'compliant', True))
        self.addPreCondition(self.getHasPropCond('JointStateController', 'rdfs:label', 'JointState', '=', 'joint_state', True))

class go_through_door(SkillBase):
    def createDescription(self):
        self.setDescription(GoThroughDoor(), self.__class__.__name__)
    
    def expand(self, skill):
        self.setProcessor(Sequential())
        skill(
            self.skill('JPMoveArm','jp_move_arm',
                specify={
                    'Arm': self.params['Arm'].value,
                    'Target': self.params['ButtonLookout'].value,
                    'Mode': self.params['JointState'].value
                }
            ),
            self.skill('ArucoEstimation', 'jp_pose_estimation', 
                specify={
                    'Camera': self.params['Camera'].value,
                    'Object': self.params['Button'].value
                }
            ),
            self.skill('ButtonPress', 'button_press',
                specify={
                    'Arm': self.params['Arm'].value,
                    'Mode': self.params['Compliant'].value,
                    'Button': self.params['Button'].value,
                    'Offset': 0.1,
                    'Force': 100
                }
            ),
            self.skill('JPMoveArm','jp_move_arm',
                specify={
                    'Arm': self.params['Arm'].value,
                    'Target': self.params['ArmHome'].value,
                    'Mode': self.params['JointState'].value
                }
            ),
            self.skill('JPDrive', 'jp_drive',
                specify={
                    'TargetLocation': self.params['TargetLocation'].value
                }
                       
            )
        )
