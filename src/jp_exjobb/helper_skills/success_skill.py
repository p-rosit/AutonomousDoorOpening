from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.params import ParamTypes
from skiros2_skill.core.skill import SkillDescription

class SuccessSkill(SkillDescription):
    def createDescription(self):
        self.addParam('msg', 'Success', ParamTypes.Required)
    
class success_skill(PrimitiveBase):
    def createDescription(self):
        self.setDescription(SuccessSkill(), self.__class__.__name__)
    
    def execute(self):
        return self.success(self.params['msg'].value)
