from skiros2_skill.core.skill import SkillBase, SkillDescription, Sequential, Loop, Selector
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

class HandEyeCalibration(SkillDescription):
    def createDescription(self):
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Required)
        self.addParam('EE', Element('scalable:Ur5EndEffector'), ParamTypes.Inferred)
        self.addParam('Gripper', Element('scalable:WsgGripper'), ParamTypes.Inferred)
        self.addParam('Camera', Element('skiros:DepthCamera'), ParamTypes.Required)

        self.addParam('Base', Element('sumo:Object'), ParamTypes.Required)
        self.addParam('Marker', Element('sumo:Object'), ParamTypes.Required)

        self.addParam('Radius', 0.7, ParamTypes.Optional)
        self.addParam('Origin Max Radius', 0.8, ParamTypes.Optional)
        self.addParam('Origin Min Radius', 0.5, ParamTypes.Optional)
        self.addParam('Max Angle (deg)', 40.0, ParamTypes.Required)
        self.addParam('Angle Interval (deg)', 10.0, ParamTypes.Optional)

        self.addParam('Velocity Threshold', 1e-4, ParamTypes.Optional)

        self.addParam('Detection Time (s)', 1.0, ParamTypes.Optional)
        self.addParam('Capture Rate (hz)', 15, ParamTypes.Optional)

        self.addParam('Max Poses', 10, ParamTypes.Required)
        self.addParam('EE has Camera', True, ParamTypes.Required)

        self.addParam('JointController', Element('scalable:ControllerState'), ParamTypes.Inferred)
        self.addParam('CompliantController', Element('scalable:ControllerState'), ParamTypes.Inferred)

        self.addPreCondition(self.getRelationCond('ArmHasEE', 'skiros:hasA', 'Arm', 'EE', True))
        self.addPreCondition(self.getRelationCond('EEhasGripper', 'skiros:hasA', 'EE', 'Gripper', True))
        self.addPreCondition(self.getPropCond('JointController', 'skiros:Value', 'JointController', '=', 'joint_config', True))
        self.addPreCondition(self.getPropCond('CompliantController', 'skiros:Value', 'CompliantController', '=', 'compliant', True))

class hand_eye_calibration(SkillBase):
    def createDescription(self):
        self.setDescription(HandEyeCalibration(), self.__class__.__name__)
    
    def expand(self, skill):
        self.setProcessor(Sequential())
        skill(
            self.skill('StartHandEyeCalibration', 'start_hand_eye_calibration'),
            self.skill(Loop(self.params['Max Poses'].value))(
                self.skill('MoveArmOnSphere', 'move_arm_on_sphere', specify={
                    'Base': self.params['Base'].value,
                    'Arm': self.params['Arm'].value,
                    'Radius': self.params['Radius'].value,
                    'Origin Max Radius': self.params['Origin Max Radius'].value,
                    'Origin Min Radius': self.params['Origin Min Radius'].value,
                    'Max Angle (deg)': self.params['Max Angle (deg)'].value,
                    'Angle Interval (deg)': self.params['Angle Interval (deg)'].value,
                    'Goal Wait (s)': 20.0
                }),
                self.skill('JPSwitchController', 'jp_switch_controller', specify={
                    'Arm': self.params['Arm'].value,
                    'Controller': self.params['JointController'].value
                }),
                self.skill('WaitForVelocity', 'wait_for_velocity', specify={
                    'Thing': self.params['Gripper'].value,
                    'Time Limit (s)': 10.0,
                    'Position Threshold': self.params['Velocity Threshold'].value,
                    'Orientation Threshold': 1e-4
                }),
                self.skill(Selector())(
                    self.skill(Sequential())(
                        self.skill('JPPoseEstimation', 'jp_pose_estimation', specify={
                            'Camera': self.params['Camera'].value,
                            'Object': self.params['Marker'].value,
                            'Detection Time (s)': self.params['Detection Time (s)'].value,
                            'Image Capture Rate (hz)': self.params['Capture Rate (hz)'].value
                        }),
                        self.skill('JPSaveHandEyeCalibData', 'jp_save_hand_eye_calib_data', specify={
                            'Hand': self.params['EE'].value,
                            'Marker': self.params['Marker'].value
                        })
                    ),
                    self.skill('SuccessSkill', 'success_skill', specify={
                        'msg': 'Marker was not visible at the specified pose.'
                    })
                )
            ),
            # self.skill('ComputeHandEyeCalibration', 'compute_hand_eye_calibration', specify={
            #     'Camera': self.params['Camera'].value,
            #     'Hand': self.params['EE'].value,
            #     'Marker': self.params['Marker'].value,
            #     'EE has camera': self.params['EE has Camera'].value
            # }),
            # self.skill('ComputeHandEyeCalibration', 'save_hand_eye_calibration', specify={
            #     'Camera': self.params['Camera'].value,
            #     'Hand': self.params['EE'].value,
            #     'Marker': self.params['Marker'].value,
            #     'EE has camera': self.params['EE has Camera'].value 
            # })
        )