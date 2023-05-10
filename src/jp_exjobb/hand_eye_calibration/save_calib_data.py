from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

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
        # publish to topic

        return self.success('Collected poses.')
