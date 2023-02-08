from skiros2_skill.core.skill import SkillDescription, SkillBase
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes
import rospy
from std_msgs.msg import Int32

class ArucoEvaluation(SkillDescription):
    def createDescription(self):
        self.addParam('ArUco marker', Element('skiros:Product'), ParamTypes.Required) 
        self.addParam('R', [0.0, 0.0, 0.0, 1.0], ParamTypes.Required) 
        self.addParam('t', [0.0, 0.0, 0.0], ParamTypes.Required) 

class aruco_evaluation(SkillBase):

    def createDescription(self):
        self.setDescription(ArucoEvaluation(), self.__class__.__name__)

    def onInit(self):
        return True

    def onPreempt(self):
        return True

    def onStart(self):
        return True

    def expand(self, skill):
        return self.success('Done')

    def onEnd(self):
        return True

    
class coordinate_comparison(PrimitiveBase):

    def createDescription(self):
        self.setDescription(ArucoEvaluation(), self.__class__.__name__)

    def onInit(self):
        return True

    def onPreempt(self):
        return True

    def onStart(self):
        return True

    def execute(self):
        #aruco = self.params['ArUco marker'].value
        #R_hat = [aruco.getProperty('skiros:OrientationW').value]
        print(self.params['R'].value)
        print(self.params['t'].value)
        return self.success('Done')

    def onEnd(self):
        return True