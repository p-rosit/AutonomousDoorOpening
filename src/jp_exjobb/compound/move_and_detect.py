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

class jp_detect_and_move_arm(SkillBase):
    """
    Summary:
        Move arm while detecting an object.

    Required Input:
        Arm:    The arm to move.
        Object: The object to detect.
        Mode:   The controller.
        Target: The target pose or joint state.

    Behaviour:
        Moves the robot arm to a pose or joint state while continuously
        detecting the object.

    Notes and Pitfalls:
        This skill works as expected but is a bit problematic. The ArUco detector
        gets a noisy estimate of where the object is if the camera is moving while
        the image is being captured. Additionally if the hand-eye calibration is
        not perfect the estimate might be even worse.
    """
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
                    'Target': self.params['Target'].value,
                    'Mode': self.params['Mode'].value
                })
            )
        )