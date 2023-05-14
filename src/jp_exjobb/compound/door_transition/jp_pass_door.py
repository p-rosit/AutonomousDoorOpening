from skiros2_skill.core.skill import SkillBase, SkillDescription, Sequential, ParallelFf
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

# class JPPassDoorTemp(SkillDescription):
#     def createDescription(self):
#         self.addParam('Heron', Element('cora:Robot'), ParamTypes.Required)
#         self.addParam('Door', Element('scalable:Door'), ParamTypes.Required)
#         self.addParam('Source', Element('scalable:Location'), ParamTypes.Inferred)
#         self.addParam('Target', Element('scalable:Location'), ParamTypes.Inferred)
#         self.addParam('SourceRegion', Element('scalable:Region'), ParamTypes.Inferred)
#         self.addParam('TargetRegion', Element('scalable:Region'), ParamTypes.Inferred)

#         self.addPreCondition(self.getRelationCond('HeronAtSource', 'skiros:at', 'Heron', 'Source', True))
#         self.addPreCondition(self.getRelationCond('HeronNotAtTarget', 'skiros:at', 'Heron', 'Target', False))
#         self.addPreCondition(self.getRelationCond('SourceInRegion', 'skiros:contain', 'SourceRegion', 'Source', True))
#         self.addPreCondition(self.getRelationCond('TargetInRegion', 'skiros:contain', 'TargetRegion', 'Target', True))
#         self.addPreCondition(self.getRelationCond('SourceInRegion', 'skiros:contain', 'SourceRegion', 'Target', False))
#         self.addPreCondition(self.getRelationCond('TargetInRegion', 'skiros:contain', 'TargetRegion', 'Source', False))

#         self.addPreCondition(self.getRelationCond('SourceRegionHasDoor', 'scalable:hasDoor', 'SourceRegion', 'Door', True))
#         self.addPreCondition(self.getRelationCond('TargetRegionHasDoor', 'scalable:hasDoor', 'TargetRegion', 'Door', True))
#         self.addPreCondition(self.getRelationCond('DoorHasWaypoint', 'skiros:hasA', 'Door', 'Target', True))

class JPPassDoor(SkillDescription):
    def createDescription(self):
        self.addParam('Heron', Element('cora:Robot'), ParamTypes.Required)
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Inferred)
        self.addParam('Door', Element('scalable:Door'), ParamTypes.Required)
        self.addParam('Source', Element('scalable:Location'), ParamTypes.Inferred)
        self.addParam('SourceRegion', Element('scalable:Region'), ParamTypes.Inferred)

        self.addPreCondition(self.getRelationCond('HeronHasArm', 'skiros:hasA', 'Heron', 'Arm', True))
        self.addPreCondition(self.getRelationCond('HeronAtSource', 'skiros:at', 'Heron', 'Source', True))
        self.addPreCondition(self.getRelationCond('RegionHasSource', 'skiros:contain', 'SourceRegion', 'Source', True))
        self.addPreCondition(self.getRelationCond('RegionHasDoor', 'scalable:hasDoor', 'SourceRegion', 'Door', True))

class jp_pass_door(SkillBase):
    def createDescription(self):
        self.setDescription(JPPassDoor(), self.__class__.__name__)
    
    def expand(self, skill):
        could_infer, msg, target = self.infer_target()

        if not could_infer:
            self.setProcessor(Sequential())
            skill(
                self.skill('FailSkill', 'fail_skill', specify={
                    'msg': msg
                })
            )
            return

        self.setProcessor(Sequential())
        skill(
            self.skill('JPArm', 'jp_arm_home', specify={
                'Arm': self.params['Arm'].value
            }),
            self.skill(ParallelFf())(
                self.skill('WatchForDoor', 'watch_for_door', specify={
                    'time_limit': 3600.0
                }),
                self.skill('JPDrive', 'jp_drive', specify={
                    'Heron': self.params['Heron'].value,
                    'TargetLocation': target
                })
            )
            # check if door closed during drive
        )
    
    def infer_target(self):
        could_infer = True
        msg = ''
        
        door = self.params['Door'].value
        region = self.params['SourceRegion'].value
        target = None

        for door_relation in door.getRelations(subj='-1', pred='skiros:hasA'):
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