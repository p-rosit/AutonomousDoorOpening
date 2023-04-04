from skiros2_skill.core.skill import SkillBase, SkillDescription, Sequential
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

class RoomNavigation(SkillDescription):
    def createDescription(self):
        self.addParam('CurrentLocation', Element('skiros:Location'), ParamTypes.Inferred)
        self.addParam('TargetLocation', Element('skiros:Location'), ParamTypes.Inferred)

        self.addParam('Robot', Element('cora:Robot'), ParamTypes.Inferred)
        self.addParam('Current', Element('scalable:Workstation'), ParamTypes.Inferred)
        self.addParam('Target', Element('scalable:Workstation'), ParamTypes.Required)

        # Pre-condition, robot is at some location but not at target
        self.addPreCondition(self.getRelationCond('LocationExists', 'skiros:contain', 'CurrentLocation', 'Current', True))
        self.addPreCondition(self.getRelationCond('PresentLocation', 'skiros:at', 'Robot', 'Current', True))
        self.addPreCondition(self.getRelationCond('RemoteExists', 'skiros:contain', 'TargetLocation', 'Target', True))
        self.addPreCondition(self.getRelationCond('RemoteLocation', 'skiros:at', 'Robot', 'Target', False))

        # Post-condition, is at target location but not at previous location
        self.addPostCondition(self.getRelationCond('MovedFromLocation', 'skiros:at', 'Robot', 'Current', False))
        self.addPostCondition(self.getRelationCond('ReachedLocation', 'skiros:at', 'Robot', 'Target', True))

class EmptySkillDescription(SkillDescription):
    def createDescription(self):
        pass

class room_navigation(SkillBase):
    def createDescription(self):
        self.setDescription(RoomNavigation(), self.__class__.__name__)
    
    def expand(self, skill):
        skill_sequence = self.determine_skill_sequence()

        skill.setProcessor(Sequential())
        skill(
            *(self.skill(skill_description, skill_name, specify=skill_parameters) for skill_description, skill_name, skill_parameters in skill_sequence)
        )

    def determine_skill_sequence(self):
        pass

    def determine_door_sequence(self):
        pass

    def connections_from(self, location):
        pass

    def find_gripper(self):
        pass

class no_valid_path_exists(PrimitiveBase):
    def createDescription(self):
        self.setDescription(EmptySkillDescription(), self.__class__.__name__)

    def execute(self):
        current_location = self.params['CurrentLocation'].value
        current = self.params['Current'].value
        target_location = self.params['TargetLocation'].value
        target = self.params['Target'].value

        msg = 'No valid path from %s in %s to %s in %s could be determined' % (current.label, current_location.label, target.label, target_location.label)

        return self.fail(msg, -1)
