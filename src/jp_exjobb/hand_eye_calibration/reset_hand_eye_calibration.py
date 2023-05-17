from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

class ResetHandEyeCalibration(SkillDescription):
    def createDescription(self):
        self.addParam(('start_hand_eye_calibration', 'started'), Element('skiros:Parameter'), ParamTypes.SharedInput)
        self.addParam(('save_hand_eye_calibration_poses', 'hand_poses'), Element('skiros:Parameter'), ParamTypes.SharedInput)
        self.addParam(('save_hand_eye_calibration_poses', 'marker_poses'), Element('skiros:Parameter'), ParamTypes.SharedInput)

class reset_hand_eye_calibration(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(ResetHandEyeCalibration(), self.__class__.__name__)
    
    def onInit(self):
        self.preempted = False
        return True

    def preStart(self):
        self.preempted = False
        return True

    def onPreempt(self):
        self.preempted = True
        return self.fail('Computation of hand eye calibration was preempted.', -1)

    def run(self):
        start = self.params['started'].value
        start.setProperty('skiros:Value', False)
        self.wmi.update_element_properties(start)

        poses = self.params['hand_poses'].value
        for pose in poses.getRelations(subj='-1', pred='skiros:hasParam'):
            self.wmi.remove_element(pose['dst'])

        poses = self.params['marker_poses'].value
        for pose in poses.getRelations(subj='-1', pred='skiros:hasParam'):
            self.wmi.remove_element(pose['dst'])

        return self.success('Hand eye calibration reset')

