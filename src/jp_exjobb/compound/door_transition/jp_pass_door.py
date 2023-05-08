from skiros2_skill.core.skill import SkillBase, SkillDescription, Sequential, ParallelFf
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

class JPPassDoor(SkillDescription):
    def createDescription(self):
        self.addParam('Heron', Element('cora:Robot'), ParamTypes.Required)
        self.addParam('Target', Element('scalable:Location'), ParamTypes.Required)
    
class jp_pass_door(SkillBase):
    def createDescription(self):
        self.setDescription(JPPassDoor(), self.__class__.__name__)
    
    def expand(self, skill):
        self.setProcessor(ParallelFf())
        skill(
            self.skill('WatchForDoor', 'watch_for_door', specify={
                'time_limit': 1000.0
            }),
            self.skill('JPDrive', 'jp_drive', specify={
                'Heron': self.params['Heron'].value,
                'Target': self.params['Target'].value
            })
        )