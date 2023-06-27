from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential, ParallelFs, ParallelFf
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

class ButtonPress(SkillDescription):
    def createDescription(self):
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Required)
        self.addParam('EE', Element('scalable:Ur5EndEffector'), ParamTypes.Inferred)
        self.addParam('Gripper', Element('scalable:WsgGripper'), ParamTypes.Inferred)
        self.addParam('LookoutPose', Element('skiros:TransformationPose'), ParamTypes.Inferred)
        self.addParam('Pose', Element('skiros:TransformationPose'), ParamTypes.Inferred)
        self.addParam('Compliant', Element('scalable:ControllerState'), ParamTypes.Inferred)
        self.addPreCondition(self.getPropCond('CompliantController', 'skiros:Value', 'Compliant', '=', 'compliant', True))

        self.addParam('Button', Element('scalable:DoorButton'), ParamTypes.Required)
        self.addParam('Offset', 0.07, ParamTypes.Optional)
        self.addParam('Pre Push Offset', 0.1, ParamTypes.Optional)
        self.addParam('Force', 100.0, ParamTypes.Optional)
        self.addParam('Sensitivity', 0.2, ParamTypes.Optional)

        self.addPreCondition(self.getRelationCond('ArmHasEE', 'skiros:hasA', 'Arm', 'EE', True))
        self.addPreCondition(self.getRelationCond('EEHasGripper', 'skiros:hasA', 'EE', 'Gripper', True))
        self.addPreCondition(self.getPropCond('ScenePose', 'skiros:Value', 'Pose', '=', 'scene_point', True))
        self.addPreCondition(self.getPropCond('LookoutPose', 'skiros:Value', 'LookoutPose', '=', 'other_scene_point', True))

class button_press(SkillBase):
    """
    Summary:
        Press a door button.

    Required Input:
        Arm:    The arm which presses the button.
        Button: The button to be pressed.

    Optional input:
        Offset:             The distance into the wall the arm tries to press in meters.
        Pre Push Offset:    The distance of the gripper from the wall before pressing.
        Force:              The magnitude of the force the gripper should feel when the
                            button is considered pressed.
        Sensitivity:        A number the force is scaled by while pressing.

    Behaviour:
        The pose of the button should be known and the button should be reachable by the
        arm when running this skill. When pressing the button the gripper is moved in the
        following way:

            Current pose saved.
            Move to pre-press pose.
            Move to press pose.
            Move to pre-press pose.
            Move to saved pose.
        
        The pre-press pose is generated from the pose of the button and is offset by the
        pre push offset in the negative z direction of the buttons frame. The press pose
        is generated in the same way but the offset is into the wall.

        The Cartesian compliant controller is used for all movements and the scalind of
        the force sensing is updated to the sensitivity right before pressing and reset
        to 1 after pressing.

    Notes and Pitfalls:
        The sensitivity should be quite small, otherwise the button pressing will make
        the gripper oscillate. The smaller the sensitivity is the smaller the offset into
        the wall can be but if the offset is too small the arm might not be able to press
        the button since detection and control is not exact.

        The sensitivity should not be zero, then the controller will be stiff and things
        might break.

        The force magnitude should be large to ensure that the button press is not stopped
        before the button actually has been pressed.

        If using Heron and the WSG gripper the gripper should be closed before executing
        this skill.
    """
    def createDescription(self):
        self.setDescription(ButtonPress(), self.__class__.__name__)

    def expand(self, skill):
        compliant = self.params['Compliant'].value

        self.setProcessor(Sequential())
        skill(
            # Save current gripper pose so we can return to it
            self.skill('SaveGripperPose', 'save_gripper_pose',
                remap={'GripperPose': 'LookoutPose'},
                specify={'Gripper': self.params['Gripper'].value}
            ),

            # Generate pre press pose
            self.skill(ParallelFf())(
                self.skill('ForceSensingOn', 'force_sensing_on',
                    specify={'Compliant': True}
                ),
                self.skill('GeneratePressPose','generate_press_pose',
                    specify={'Offset': -self.params['Pre Push Offset'].value}
                )
            ),
            # Move to pre press pose
            self.skill(ParallelFs())(
                self.skill('TimerSkill', 'timer_skill', specify={
                    'Max Time (s)': 20.0
                }),
                self.skill('JPMoveArm','jp_move_arm',
                    remap={'Target': 'Pose'},
                    specify={'Mode': compliant}
                )
            ),

            # Generate press pose
            self.skill(ParallelFf())(
                self.skill('GeneratePressPose','generate_press_pose',
                    specify={'Offset': self.params['Offset'].value}
                ),
                self.skill('ScaleUpdate', 'scale_update',
                    specify={'Scale': self.params['Sensitivity'].value}
                )
            ),
            # Move to press pose
            self.skill(ParallelFs())(
                self.skill('JPMoveArm','jp_move_arm',
                    remap={'Target': 'Pose'},
                    specify={'Mode': compliant}
                ),
                self.skill('WaitForForce', 'wait_for_force',
                    specify={'Force': self.params['Force'].value}
                )
            ),
            # Check if force goal was met
            self.skill('ForceCheck', 'force_check'),
            
            # Generate pre press pose
            self.skill(ParallelFf())(
                self.skill('ScaleUpdate', 'scale_update',
                    specify={'Scale': 1.0}
                ),
                self.skill('GeneratePressPose','generate_press_pose',
                    specify={'Offset': -self.params['Pre Push Offset'].value}
                )
            ),
            # Move back to pre press pose
            self.skill(ParallelFs())(
                self.skill('TimerSkill', 'timer_skill', specify={
                    'Max Time (s)': 20.0
                }),
                self.skill('JPMoveArm','jp_move_arm',
                    remap={'Target': 'Pose'},
                    specify={'Mode': compliant}
                )
            ),

            # Move back to lookout pose
            self.skill(ParallelFs())(
                self.skill('TimerSkill', 'timer_skill', specify={
                    'Max Time (s)': 20.0
                }),
                self.skill('JPMoveArm','jp_move_arm',
                    remap={'Target': 'LookoutPose'},
                    specify={'Mode': compliant}
                )
            )
        )

class GeneratePressPose(SkillDescription):
    def createDescription(self):
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Required)
        self.addParam('EE', Element('scalable:Ur5EndEffector'), ParamTypes.Inferred)
        self.addParam('Gripper', Element('scalable:WsgGripper'), ParamTypes.Inferred)
        self.addParam('Pose', Element('skiros:TransformationPose'), ParamTypes.Inferred)

        self.addParam('Button', Element('sumo:Object'), ParamTypes.Required)
        self.addParam('Offset', 0.0, ParamTypes.Required)

        self.addPreCondition(self.getRelationCond('ArmHasEE', 'skiros:hasA', 'Arm', 'EE', True))
        self.addPreCondition(self.getRelationCond('EEHasGripper', 'skiros:hasA', 'EE', 'Gripper', True))
        self.addPreCondition(self.getPropCond('ScenePose', 'skiros:Value', 'Pose', '=', 'scene_point', True))

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
