from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

class SaveHandEyeCalibration(SkillDescription):
    def createDescription(self):
        self.addParam(('start_hand_eye_calibration', 'ee_has_camera'), Element('skiros:Parameter'), ParamTypes.SharedInput)
        self.addParam(('compute_hand_eye_calibration', 'computed_transformation'), Element('skiros:TransformationPose'), ParamTypes.SharedInput)

class save_hand_eye_calibration(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(SaveHandEyeCalibration(), self.__class__.__name__)

    def run(self):
        camera_pose = self.params['computed_transformation'].value
        ee_has_camera = self.params['ee_has_camera'].value.getProperty('skiros:Value').value

        print(camera_pose.getData(':Pose'))
        print(ee_has_camera)
        return self.success('Hand eye calibration completed')
