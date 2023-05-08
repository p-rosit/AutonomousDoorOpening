from skiros2_skill.core.skill import SkillBase, SkillDescription, Sequential
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

class JPPassElevator(SkillDescription):
    def createDescription(self):
        self.addParam('Heron', Element('cora:Robot'), ParamTypes.Required)
        self.addParam('Elevator', Element('scalable:Elevator'), ParamTypes.Required)
        self.addParam('Source', Element('scalable:Location'), ParamTypes.Inferred)
        self.addParam('Target', Element('scalable:Location'), ParamTypes.Inferred)
        self.addParam('SourceRegion', Element('scalable:Region'), ParamTypes.Inferred)
        self.addParam('TargetRegion', Element('scalable:Region'), ParamTypes.Inferred)

        self.addPreCondition(self.getRelationCond('HeronAtSource', 'skiros:at', 'Heron', 'Source', True))
        self.addPreCondition(self.getRelationCond('HeronNotAtTarget', 'skiros:at', 'Heron', 'Target', False))
        self.addPreCondition(self.getRelationCond('SourceInRegion', 'skiros:contain', 'SourceRegion', 'Source', True))
        self.addPreCondition(self.getRelationCond('TargetInRegion', 'skiros:contain', 'TargetRegion', 'Target', True))
        # self.addPreCondition(self.getRelationCond('SourceInRegion', 'skiros:contain', 'SourceRegion', 'Target', False))
        # self.addPreCondition(self.getRelationCond('TargetInRegion', 'skiros:contain', 'TargetRegion', 'Source', False))

        self.addPreCondition(self.getRelationCond('SourceRegionHasDoor', 'scalable:hasDoor', 'SourceRegion', 'Elevator', True))
        self.addPreCondition(self.getRelationCond('TargetRegionHasDoor', 'scalable:hasDoor', 'TargetRegion', 'Elevator', True))
        self.addPreCondition(self.getRelationCond('DoorHasWaypoint', 'skiros:hasA', 'Elevator', 'Target', True))

class jp_pass_elevator(SkillBase):
    def createDescription(self):
        self.setDescription(JPPassElevator(), self.__class__.__name__)
    
    def expand(self, skill):
        self.setProcessor(Sequential())
        target_region = self.params['TargetRegion'].value

        if target_region.type == 'scalable:Region':
            skill(
                self.skill('FailSkill', 'fail_skill', specify={
                    'msg': 'Go from room to elevator'
                })
            )
        elif target_region.type == 'scalable:ElevatorRegion':
            skill(
                self.skill('FailSkill', 'fail_skill', specify={
                    'msg': 'Go from elevator to room'
                })
            )
        else:
            raise RuntimeError('Cannot handle region type.')
