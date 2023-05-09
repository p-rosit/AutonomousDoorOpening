from skiros2_skill.core.skill import SkillBase, SkillDescription, Sequential
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

class JPPassElevatorTemp(SkillDescription):
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
        self.addPreCondition(self.getRelationCond('SourceInRegion', 'skiros:contain', 'SourceRegion', 'Target', False))
        self.addPreCondition(self.getRelationCond('TargetInRegion', 'skiros:contain', 'TargetRegion', 'Source', False))

        self.addPreCondition(self.getRelationCond('SourceRegionHasDoor', 'scalable:hasDoor', 'SourceRegion', 'Elevator', True))
        self.addPreCondition(self.getRelationCond('TargetRegionHasDoor', 'scalable:hasDoor', 'TargetRegion', 'Elevator', True))
        self.addPreCondition(self.getRelationCond('DoorHasWaypoint', 'skiros:hasA', 'Elevator', 'Target', True))

class JPPassElevator(SkillDescription):
    def createDescription(self):
        self.addParam('Heron', Element('cora:Robot'), ParamTypes.Required)
        self.addParam('Elevator', Element('scalable:Elevator'), ParamTypes.Required)
        self.addParam('Source', Element('scalable:Location'), ParamTypes.Inferred)
        self.addParam('SourceRegion', Element('scalable:Region'), ParamTypes.Inferred)

        self.addPreCondition(self.getRelationCond('HeronAtSource', 'skiros:at', 'Heron', 'Source', True))
        self.addPreCondition(self.getRelationCond('RegionHasSource', 'skiros:contain', 'SourceRegion', 'Source', True))
        self.addPreCondition(self.getRelationCond('RegionHasElevator', 'scalable:hasDoor', 'SourceRegion', 'Elevator', True))

class jp_pass_elevator(SkillBase):
    def createDescription(self):
        self.setDescription(JPPassElevator(), self.__class__.__name__)
    
    def expand(self, skill):
        self.setProcessor(Sequential())
        could_infer, msg, target = self.infer_target()

        if not could_infer:
            self.setProcessor(Sequential())
            skill(
                self.skill('FailSkill', 'fail_skill', specify={
                    'msg': msg
                })
            )
            return
        
        could_infer, msg, target_region = self.infer_region(target)

        if not could_infer:
            self.setProcessor(Sequential())
            skill(
                self.skill('FailSkill', 'fail_skill', specify={
                    'msg': msg
                })
            )
            return

        if target_region.type == 'scalable:Region':
            skill(
                self.skill('JPDrive', 'jp_move_heron', specify={
                    'Heron': self.params['Heron'].value,
                    'TargetLocation': target
                }),
                self.skill('SuccessSkill', 'success_skill', specify={
                    'msg': 'Go from room to elevator'
                })
            )
        elif target_region.type == 'scalable:ElevatorRegion':
            skill(
                self.skill('JPDrive', 'jp_move_heron', specify={
                    'Heron': self.params['Heron'].value,
                    'TargetLocation': target
                }),
                self.skill('SuccessSkill', 'success_skill', specify={
                    'msg': 'Go from elevator to room'
                })
            )
        else:
            raise RuntimeError('Cannot handle region type.')

    def infer_target(self):
        could_infer = True
        msg = ''
        
        elevator = self.params['Elevator'].value
        region = self.params['SourceRegion'].value
        target = None

        for door_relation in elevator.getRelations(subj='-1', pred='skiros:hasA'):
            obj = self.wmi.get_element(door_relation['dst'])

            obj_type = self.wmi.get_super_class(obj.type)
            obj_region = obj.getRelations(subj=region.id, pred='skiros:contain', obj='-1')
            if obj_type != 'scalable:Location' or len(obj_region) > 0:
                continue

            if target is not None:
                could_infer = False
                msg = 'Target can not be uniquely determined. Got "%s" and "%s" as possible candidates.' % (target.id, obj.id)
                break

            target = obj

        if target is None:
            could_infer = False
            msg = 'Target not found.'

        return could_infer, msg, target
    
    def infer_region(self, target):
        could_infer = True
        msg = ''
        region = None

        for target_relation in target.getRelations(pred='skiros:contain', obj='-1'):
            obj = self.wmi.get_element(target_relation['src'])

            if obj.type != 'scalable:Region' and self.wmi.get_super_class(obj.type) != 'scalable:Region':
                continue

            if region is not None:
                could_infer = False
                msg = 'Region can not be uniquely determined. Got "%s" and "%s" as possible candidates.' % (region.id, obj.id)
                break
            
            region = obj

        if region is None:
            could_infer = False
            msg = 'Target region not found.'
         
        return could_infer, msg, region