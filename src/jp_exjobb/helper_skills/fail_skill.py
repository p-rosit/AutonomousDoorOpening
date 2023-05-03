from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.params import ParamTypes

class FailSkill(SkillDescription):
    def createDescription(self):
        self.addParam('msg', 'Failed successfully.', ParamTypes.Required)

class fail_skill(PrimitiveBase):
    def createDescription(self):
        self.setDescription(FailSkill(), self.__class__.__name__)
    
    def execute(self):
        return self.fail(self.params['msg'].value, -1)
