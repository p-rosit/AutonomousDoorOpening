from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.params import ParamTypes

class FailSkill(SkillDescription):
    def createDescription(self):
        self.addParam('msg', 'Failed successfully.', ParamTypes.Required)

class fail_skill(PrimitiveBase):
    """
    Summary:
        Skill which always fails.

    Required Input:
        msg: The message diplayed in SkiROS

    Behaviour:
        This skill is meant to always fail. It should be used as a way
        to control how a skill executes. Sometimes you want a compouned
        skill to fail if some condition is true and sometimes you want
        it to succeed if the same condition is true.
    """
    def createDescription(self):
        self.setDescription(FailSkill(), self.__class__.__name__)
    
    def execute(self):
        return self.fail(self.params['msg'].value, -1)
