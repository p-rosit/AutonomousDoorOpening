from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

class ComputeHandEyeCalibration(SkillDescription):
    def createDescription(self):
        self.addParam('Camera', Element('skiros:DepthCamera'), ParamTypes.Required)
        self.addParam('Hand', Element('sumo:Object'), ParamTypes.Required)
        self.addParam('Marker', Element('sumo:Object'), ParamTypes.Required)
        self.addParam('EE has camera', True, ParamTypes.Required)

class compute_hand_eye_calibration(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(ComputeHandEyeCalibration(), self.__class__.__name__)

    def run(self):
        return self.success(':)')

class save_hand_eye_calibration(PrimitiveBase):
    def createDescription(self):
        self.setDescription(ComputeHandEyeCalibration(), self.__class__.__name__)
    
    def execute(self):
        return self.success(':)')
