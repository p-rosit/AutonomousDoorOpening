from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

class DoorClosed(SkillDescription):
    def createDescription(self):
        self.addParam(('watch_for_door', 'door_state_changed'), Element('skiros:Parameter'), ParamTypes.SharedInput)

class DoorOpened(SkillDescription):
    def createDescription(self):
        self.addParam(('watch_for_door', 'door_state_changed'), Element('skiros;:Parameter'), ParamTypes.SharedInput)

class door_closed(PrimitiveBase):
    def createDescription(self):
        self.setDescription(DoorClosed(), self.__class__.__name__)
    
    def execute(self):
        state_changed = self.params['door_state_changed'].value
        door_closed = state_changed.getProperty('skiros:Value').value

        if door_closed:
            return self.fail('Door closed.', -1)
        return self.success('Door stayed open.')

class door_opened(PrimitiveBase):
    def createDescription(self):
        self.setDescription(DoorOpened(), self.__class__.__name__)
    
    def execute(self):
        state_changed = self.params['door_state_changed'].value
        door_opened = state_changed.getProperty('skiros:Value').value
        
        if door_opened:
            return self.success('Door opened.')
        return self.fail('Door stayed closed.', -1)
