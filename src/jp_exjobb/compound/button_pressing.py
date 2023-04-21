from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential, ParallelFs
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

class ButtonPress(SkillDescription):
    def createDescription(self):
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Required)
        self.addParam('Gripper', Element('scalable:WsgGripper'), ParamTypes.Inferred)
        self.addParam('Pose', Element('skiros:TransformationPose'), ParamTypes.Inferred)
        self.addParam('Compliant', Element('scalable:ControllerState'), ParamTypes.Inferred)
        self.addPreCondition(self.getPropCond('CompliantController', 'skiros:Value', 'Compliant', '=', 'compliant', True))

        self.addParam('Button', Element('scalable:DoorButton'), ParamTypes.Required)
        self.addParam('Offset', 0.1, ParamTypes.Required)
        self.addParam('Force', 100.0, ParamTypes.Required)

        self.addPreCondition(self.getRelationCond('ArmHasGripper', 'skiros:hasA', 'Arm', 'Gripper', True))
        self.addPreCondition(self.getRelationCond('GripperHasPose', 'skiros:contain', 'Gripper', 'Pose', True))

class button_press(SkillBase):
    def createDescription(self):
        self.setDescription(ButtonPress(), self.__class__.__name__)

    def expand(self, skill):
        self.setProcessor(Sequential())
        skill(
            # Move to lookout pose
            self.skill('ForceSensingOn', 'force_sensing_on', specify={'Compliant': True}),
            self.skill('GeneratePressPose','generate_press_pose', specify={'Offset': -0.1}),
            self.skill('JPMoveArm','jp_move_arm', remap={'Target': 'Pose'}, specify={'Mode': self.params['Compliant'].value}),

            # Move to pre press pose
            self.skill('GeneratePressPose','generate_press_pose', specify={'Offset': self.params['Offset'].value}),
            self.skill(ParallelFs())(
                # Move to press pose
                self.skill('JPMoveArm','jp_move_arm', remap={'Target': 'Pose'}, specify={'Mode': self.params['Compliant'].value}),
                self.skill('WaitForForce', 'wait_for_force', specify={'Force': self.params['Force'].value})
            ),
            self.skill('ForceCheck', 'force_check'),
            
            # Move back to pre press pose
            self.skill('GeneratePressPose','generate_press_pose', specify={'Offset': -0.1}),
            self.skill('JPMoveArm','jp_move_arm', remap={'Target': 'Pose'}, specify={'Mode': self.params['Compliant'].value})
        )

class GeneratePressPose(SkillDescription):
    def createDescription(self):
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Required)
        self.addParam('Gripper', Element('scalable:WsgGripper'), ParamTypes.Inferred)
        self.addParam('Pose', Element('skiros:TransformationPose'), ParamTypes.Inferred)

        self.addParam('Button', Element('sumo:Object'), ParamTypes.Required)
        self.addParam('Offset', 0.0, ParamTypes.Required)

        self.addPreCondition(self.getRelationCond('ArmHasGripper', 'skiros:hasA', 'Arm', 'Gripper', True))
        self.addPreCondition(self.getRelationCond('GripperHasPose', 'skiros:contain', 'Gripper', 'Pose', True))

class generate_press_pose(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(GeneratePressPose(), self.__class__.__name__)
    
    def run(self):
        gripper = self.params['Gripper'].value
        button = self.params['Button'].value
        pose = self.params['Pose'].value    

        gripper_offset = gripper.getProperty('skiros:SizeZ').value
        offset = self.params['Offset'].value

        pose.setProperty('skiros:BaseFrameId', button.getProperty('skiros:FrameId').value)
        pose.setProperty('skiros:PositionX', 0.0)
        pose.setProperty('skiros:PositionY', 0.0)
        pose.setProperty('skiros:PositionZ', offset - gripper_offset)
        pose.setProperty('skiros:OrientationX', 0.0)
        pose.setProperty('skiros:OrientationY', 0.0)
        pose.setProperty('skiros:OrientationZ', 1.0)
        pose.setProperty('skiros:OrientationW', 0.0)
        self.wmi.update_element_properties(pose)

        return self.success('Pose generated.')
