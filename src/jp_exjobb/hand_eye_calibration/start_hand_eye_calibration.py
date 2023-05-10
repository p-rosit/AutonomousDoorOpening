from skiros2_common.core.primitive import PrimitiveBase
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

class StartHandEyeCalibration(SkillDescription):
    def createDescription(self):
        pass

class start_hand_eye_calibration(PrimitiveBase):
    def createDescription(self):
        self.setDescription(StartHandEyeCalibration(), self.__class__.__name__)
    
    def execute(self):
        return self.success(':)')