from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.params import ParamTypes
from skiros2_skill.core.skill import SkillDescription

class SuccessSkill(SkillDescription):
    def createDescription(self):
        self.addParam('msg', 'Success', ParamTypes.Required)
    
class success_skill(PrimitiveBase):
    """
    Summary:
        Skill which always succeeds.

    Required Input:
        msg: The message diplayed in SkiROS

    Behaviour:
        This skill is meant to always succeed. It should be used as a way
        to control how a skill executes. Sometimes you want a compouned
        skill to fail if some condition is true and sometimes you want
        it to succeed if the same condition is true.
    """
    def createDescription(self):
        self.setDescription(SuccessSkill(), self.__class__.__name__)
    
    def execute(self):
        return self.success(self.params['msg'].value)
