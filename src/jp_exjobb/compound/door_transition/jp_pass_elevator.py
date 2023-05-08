from skiros2_skill.core.skill import SkillBase, SkillDescription, Sequential
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

class JPPassElevator(SkillDescription):
    def createDescription(self):
        self.addParam('Heron', Element('cora:Robot'), ParamTypes.Required)
        self.addParam('Elevator', Element('scalable:Elevator'), ParamTypes.Required)
        self.addParam('Source', Element('scalable:Location'), ParamTypes.Inferred)
        self.addParam('Target', Element('scalable:Location'), ParamTypes.Inferred)

# class jp_pass_elevator(SkillBase):
#     def createDescription(self):
#         self.setDescription(JPPassElevator(), self.__class__.__name__)
    
#     def expand(self, skill):
#         self.setProcessor(Sequential())
#         skill(

#         )
