from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import  ParamTypes

class StartHandEyeCalibration(SkillDescription):
    def createDescription(self):
        self.addParam('RobotBase',      Element('sumo:Object'),         ParamTypes.Required)
        self.addParam('Hand',           Element('sumo:Object'),         ParamTypes.Required)
        self.addParam('Camera',         Element('skiros:DepthCamera'),  ParamTypes.Required)
        self.addParam('Marker',         Element('sumo:Object'),         ParamTypes.Required)
        self.addParam('Base',           Element('sumo:Object'),         ParamTypes.Required)
        self.addParam('EE has Camera',  True,                           ParamTypes.Required)

        self.addParam('started',        Element('skiros:Parameter'),    ParamTypes.SharedOutput)
        self.addParam('hand',           Element('sumo:Object'),         ParamTypes.LinkedOutput)
        self.addParam('camera',         Element('skiros:DepthCamera'),  ParamTypes.LinkedOutput)
        self.addParam('marker',         Element('sumo:Object'),         ParamTypes.LinkedOutput)
        self.addParam('base',           Element('sumo:Object'),         ParamTypes.LinkedOutput)
        self.addParam('ee_has_camera',  Element('skiros:Parameter'),    ParamTypes.SharedOutput)

class start_hand_eye_calibration(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(StartHandEyeCalibration(), self.__class__.__name__)
    
    def run(self):
        start = self.params['started'].value

        if start.hasProperty('skiros:Value', not_none=True):
            started = start.getProperty('skiros:Value').value
            if started:
                return self.fail('Hand eye calibration has already been started.', -1)

        self.status = 'Saving parameters.'

        start.setProperty('skiros:Value', True)
        self.setOutput('started', start)

        self.setOutput('hand', self.params['Hand'].value)
        self.setOutput('camera', self.params['Camera'].value)
        self.setOutput('marker', self.params['Marker'].value)
        self.setOutput('base', self.params['Base'].value)
        
        ee_has_camera = self.params['ee_has_camera'].value
        ee_has_camera.setProperty('skiros:Value', self.params['EE has Camera'].value)
        self.setOutput('ee_has_camera', ee_has_camera)

        return self.success('Hand eye calibration started.')
