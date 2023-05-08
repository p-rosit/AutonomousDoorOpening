from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_skill.core.skill import SkillBase, SkillDescription, Sequential, ParallelFf
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

class WatchForDoor(SkillDescription):
    def createDescription(self):
        self.addParam('time_limit', 1000, ParamTypes.Required)

class watch_for_door(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(WatchForDoor(), self.__class__.__name__)

    def expand(self, skill):
        return super().expand(skill)

