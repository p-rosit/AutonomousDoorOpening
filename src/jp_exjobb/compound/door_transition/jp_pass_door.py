from skiros2_skill.core.skill import SkillBase, SkillDescription, Sequential, ParallelFf
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

class JPPassDoor(SkillDescription):
    def createDescription(self):
        self.addParam('Heron', Element('cora:Robot'), ParamTypes.Required)
        self.addParam('Door', Element('scalable:Door'), ParamTypes.Required)
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

        self.addPreCondition(self.getRelationCond('SourceRegionHasDoor', 'scalable:hasDoor', 'SourceRegion', 'Door', True))
        self.addPreCondition(self.getRelationCond('TargetRegionHasDoor', 'scalable:hasDoor', 'TargetRegion', 'Door', True))
        self.addPreCondition(self.getRelationCond('DoorHasWaypoint', 'skiros:hasA', 'Door', 'Target', True))

class jp_pass_door(SkillBase):
    def createDescription(self):
        self.setDescription(JPPassDoor(), self.__class__.__name__)
    
    def expand(self, skill):
        # Don't bother with the WM preconditions...
        
        self.setProcessor(ParallelFf())
        skill(
            self.skill('FailSkill', 'fail_skill', specify={
                'msg': 'Going through door...'
            })
            # self.skill('WatchForDoor', 'watch_for_door', specify={
            #     'time_limit': 1000.0
            # }),
            # self.skill('JPDrive', 'jp_drive', specify={
            #     'Heron': self.params['Heron'].value,
            #     'Target': self.params['Target'].value
            # })
        )