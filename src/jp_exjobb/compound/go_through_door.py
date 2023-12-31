from skiros2_skill.core.skill import SkillBase, SkillDescription, Sequential
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

class GoThroughDoor(SkillDescription):
    def createDescription(self):
        self.addParam('Heron', Element('cora:Robot'), ParamTypes.Required)
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Inferred)
        self.addParam('EE', Element('scalable:Ur5EndEffector'), ParamTypes.Inferred)
        self.addParam('Gripper', Element('scalable:WsgGripper'), ParamTypes.Inferred)
        self.addParam('Camera', Element('skiros:DepthCamera'), ParamTypes.Inferred)
        self.addPreCondition(self.getRelationCond('HeronHasArm', 'skiros:hasA', 'Heron', 'Arm', True))
        self.addPreCondition(self.getRelationCond('ArmHasEE', 'skiros:hasA', 'Arm', 'EE', True))
        self.addPreCondition(self.getRelationCond('EEHasGripper', 'skiros:hasA', 'EE', 'Gripper', True))
        self.addPreCondition(self.getRelationCond('EEhasCamera', 'skiros:hasA', 'EE', 'Camera', True))

        self.addParam('Button', Element('scalable:DoorButton'), ParamTypes.Required)
        self.addParam('ButtonLookout', Element('scalable:JointState'), ParamTypes.Inferred)
        self.addPreCondition(self.getRelationCond('ButtonHasLookout', 'skiros:hasA', 'Button', 'ButtonLookout', True))

        self.addParam('TargetLocation', Element('scalable:Waypoint'), ParamTypes.Required)
        self.addParam('ArmHome', Element('scalable:JointState'), ParamTypes.Inferred)
        self.addParam('JointState', Element('scalable:ControllerState'), ParamTypes.Inferred)
        self.addParam('Compliant', Element('scalable:ControllerState'), ParamTypes.Inferred)
        self.addPreCondition(self.getPropCond('HomePosition', 'skiros:Value', 'ArmHome', '=', 'home_state', True))
        self.addPreCondition(self.getPropCond('CompliantController', 'skiros:Value', 'Compliant', '=', 'compliant', True))
        self.addPreCondition(self.getPropCond('JointStateController', 'skiros:Value', 'JointState', '=', 'joint_config', True))

        self.addParam('Offset', 0.05, ParamTypes.Required)

class go_through_door(SkillBase):
    """
    Summary:
        Open a door and go through it.

    Required Input:
        Heron:          The robot.
        Button:         The button which opens the door.
        TargetLocation: The target location just past the door.

    Behaviour:
        Finds the button, presses it. Moves the arm to the home position.
        Goes through the door.
    """
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
            self.skill('JPPoseEstimation', 'jp_pose_estimation', 
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
                    'Offset': self.params['Offset'].value,
                    'Force': 60.0
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
