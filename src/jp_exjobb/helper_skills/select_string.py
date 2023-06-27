from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.params import ParamTypes

class SelectString(SkillDescription):
    def createDescription(self):
        self.addParam('Selection', '', ParamTypes.Required)
        self.addParam('Input', '', ParamTypes.Required)

class select_string(PrimitiveBase):
    """
    Summary:
        Selects a string
    
    Required Input:
        Selection:  The correct value
        Input:      The input

    Behaviour:
        If the input is equal to the selection the skill succeeds,
        otherwise the skill fails.

    Notes and Pitfalls:
        This skill is supposed to be used in conjunction with the selector
        skillprocessor which enables one to write if statements in
        behaviour trees.
    """
    def createDescription(self):
        self.setDescription(SelectString(), self.__class__.__name__)

    def execute(self):
        selection = self.params['Selection'].value
        input =  self.params['Input'].value

        if selection == input:
            return self.success('%s chosen.' % selection)
        return self.fail('%s not chosen.' % selection, -1)
