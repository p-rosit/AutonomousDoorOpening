from skiros2_common.core.primitive import PrimitiveBase
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element


class WriteSimCamParams(SkillDescription):
    def createDescription(self):
        self.addParam('Camera', Element('skiros:DepthCamera'), ParamTypes.Required)
        self.addParam('Camera Parameters', Element('scalable:CalibrationParameters'), ParamTypes.Inferred)
        self.addPreCondition(self.getRelationCond('HasCalibrationParameters', 'skiros:hasA', 'Camera', 'Camera Parameters', True))

class write_sim_cam_params(PrimitiveBase):
    def createDescription(self):
        self.setDescription(WriteSimCamParams(), self.__class__.__name__)
    
    def execute(self):
        cam_pam = self.params['Camera Parameters'].value

        cam_pam.setData(':CameraCalibrationParameters', ((
                585.756070948,
                579.430235849,
                319.5,
                239.5
            ), (
                0.0,
                0.0,
                0.0,
                0.0,
                0.0
            )
        ))

        self.wmi.update_element_properties(cam_pam)

        return self.success('Update camera parameters to simulation parameters')
