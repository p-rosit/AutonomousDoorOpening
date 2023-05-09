from skiros2_skill.core.skill import SkillBase, SkillDescription, Sequential
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

class HandEyeCalibFindMarker(SkillDescription):
    def createDescription(self):
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Required)
        self.addParam('EE', Element('scalable:Ur5EndEffector'), ParamTypes.Inferred)
        self.addParam('Camera', Element('skiros:DepthCamera'), ParamTypes.Inferred)

        self.addParam('Marker', Element('sumo:Object'), ParamTypes.Required)
        self.addParam('Detection Time (s)', 1.0, ParamTypes.Required)
        self.addParam('Capture Rate (hz)', 15, ParamTypes.Optional)

        self.addPreCondition(self.getRelationCond('ArmHasEE', 'skiros:hasA', 'Arm', 'EE', True))
        self.addPreCondition(self.getRelationCond('EEHasCamera', 'skiros:hasA', 'EE', 'Camera', True))

class hand_eye_calib_find_marker(SkillBase):
    def createDescription(self):
        self.setDescription(HandEyeCalibFindMarker(), self.__class__.__name__)
    
    def expand(self, skill):
        self.setProcessor(Sequential())
        skill(
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
        )

class JPSaveHandEyeCalibData(SkillDescription):
    def createDescription(self):
        self.addParam('Hand', Element('sumo:Object'), ParamTypes.Required)
        self.addParam('Marker', Element('sumo:Object'), ParamTypes.Required)

class jp_save_hand_eye_calib_data(PrimitiveBase):
    def createDescription(self):
        self.setDescription(JPSaveHandEyeCalibData(), self.__class__.__name__)

    def onStart(self):
        hand = self.params['Hand'].value
        marker = self.params['Marker'].value

        if not hand.hasData(':Pose'):
            return self.startError('Hand does not have a pose.', -1)
    
        if not marker.hasData(':Pose'):
            return self.startError('Marker does not have a pose', -1)

        return True

    def execute(self):
        hand = self.params['Hand'].value
        marker = self.params['Marker'].value

        hand_pose = hand.getData(':Pose')
        marker_pose = marker.getData(':Pose')

        return self.success('Collected poses.')
