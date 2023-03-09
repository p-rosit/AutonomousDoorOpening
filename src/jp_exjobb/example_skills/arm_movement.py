from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

class JPMoveArm(SkillDescription):
    def createDescription(self):
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Required)

class jp_move_arm(PrimitiveBase):

    def createDescription(self):
        self.setDescription(JPMoveArm(), self.__class__.__name__)

    def onInit(self):
        return True

    def onPreempt(self):
        return True
    
    def onStart(self):
        return True

    def execute(self):
        return self.success('yay')

    def onEnd(self):
        return True
