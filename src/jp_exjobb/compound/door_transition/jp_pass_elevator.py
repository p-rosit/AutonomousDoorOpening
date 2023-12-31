from skiros2_skill.core.skill import SkillBase, SkillDescription, Sequential, RetryOnFail, InferInvalid, ParallelFf
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

class JPPassElevator(SkillDescription):
    def createDescription(self):
        self.addParam('Heron', Element('cora:Robot'), ParamTypes.Required)
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Inferred)
        self.addParam('Elevator', Element('scalable:Elevator'), ParamTypes.Required)
        self.addParam('Source', Element('scalable:Location'), ParamTypes.Inferred)
        self.addParam('SourceRegion', Element('scalable:Region'), ParamTypes.Inferred)

        self.addPreCondition(self.getRelationCond('HeronHasArm', 'skiros:hasA', 'Heron', 'Arm', True))
        self.addPreCondition(self.getRelationCond('HeronAtSource', 'skiros:at', 'Heron', 'Source', True))
        self.addPreCondition(self.getRelationCond('RegionHasSource', 'skiros:contain', 'SourceRegion', 'Source', True))
        self.addPreCondition(self.getRelationCond('RegionHasElevator', 'scalable:hasDoor', 'SourceRegion', 'Elevator', True))

class jp_pass_elevator(SkillBase):
    """
    Summary:
        Pass an elevator door with a mobile robot.

    Required Input:
        Heron:      The robot.
        Elevator:   The elevator door to pass.

    Behaviour:
        Unused so far. You figure it out.
    """
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
                self.skill('JPArm', 'jp_arm_home'),
                self.skill(RetryOnFail(Sequential()))(
                    self.skill('DetectDoorState', 'watch_for_door', specify={
                        'Region Transition': self.params['Elevator'].value,
                        'Time Limit (s)': 3600.0,
                        'Fail On Close': False
                    }),
                    self.skill('DetectDoorState', 'wait_for_door', specify={
                        'Region Transition': self.params['Elevator'].value,
                        'Time Limit (s)': 3600.0
                    })
                    # detect what floor we are on and if it is the correct floor
                ),
                # switch map
                self.skill('JPDrive', 'jp_move_heron', specify={
                    'Heron': self.params['Heron'].value,
                    'TargetLocation': target
                })
            )
        elif target_region.type == 'scalable:ElevatorRegion':
            skill(
                self.skill('JPArm', 'jp_arm_home'),
                self.skill('DetectDoorState', 'wait_for_door', specify={
                    'Region Transition': self.params['Elevator'].value,
                    'Time Limit (s)': 3600.0
                }),
                self.skill('DoorOpened', 'door_opened'),
                self.skill(ParallelFf())(
                    self.skill('DetectDoorState', 'watch_for_door', specify={
                        'Region Transition': self.params['Elevator'].value,
                        'Time Limit (s)': 3600.0,
                        'Fail On Close': True
                    }),
                    self.skill('JPDrive', 'jp_move_heron', specify={
                        'Heron': self.params['Heron'].value,
                        'TargetLocation': target
                    })
                ),
                self.skill('DoorClosed', 'door_closed')
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
            if obj.type == 'scalable:RegionBB' or obj_type != 'scalable:Location' or len(obj_region) > 0:
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