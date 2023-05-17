from skiros2_skill.core.skill import SkillBase, SkillDescription, Sequential
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

from queue import Queue

class NavigateBuilding(SkillDescription):
    def createDescription(self):
        self.addParam('Heron', Element('cora:Robot'), ParamTypes.Required)
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Inferred)
        self.addPreCondition(self.getRelationCond('HeronHasArm', 'skiros:hasA', 'Heron', 'Arm', True))

        self.addParam('Source', Element('scalable:Location'), ParamTypes.Inferred)
        self.addParam('Destination', Element('scalable:Location'), ParamTypes.Required)

        self.addParam('SourceRegion', Element('scalable:Region'), ParamTypes.Inferred)
        self.addParam('DestinationRegion', Element('scalable:Region'), ParamTypes.Inferred)

        self.addPreCondition(self.getRelationCond('HeronAtWorkstation', 'skiros:at', 'Heron', 'Source', True))
        self.addPreCondition(self.getRelationCond('SourceInRegion', 'skiros:contain', 'SourceRegion', 'Source', True))
        self.addPreCondition(self.getRelationCond('DestinationInRegion', 'skiros:contain', 'DestinationRegion', 'Destination', True))

class navigate_building(SkillBase):
    def createDescription(self):
        self.setDescription(NavigateBuilding(), self.__class__.__name__)
    
    def expand(self, skill):
        self.setProcessor(Sequential())

        planning_succeded, path = self.plan_path()

        if planning_succeded:
            skill_list = self.build_skill_list(path)
        else:
            skill_list = self.skill('FailSkill', 'fail_skill', specify={
                'msg': 'Path to goal from "%s" to "%s" does not exist.' % (self.params['Source'].value.label, self.params['Destination'].value.label)
            })

        for sk in skill_list:
            print(sk.label)

        skill(*skill_list)
        # skill(self.skill('FailSkill', 'fail_skill', specify={'msg': 'no'}))

    def build_skill_list(self, path):
        arm = self.params['Arm'].value
        skill_list = []

        # for ii in path:
        #     (_, b), (_, d), sm = ii
        #     print((b, d, sm))

        for node in path:
            (button_waypoint, dom), (door_waypoint, door), region = node

            # Go to button waypoint
            skill_list.append(self.skill('JPDrive', 'jp_drive',
                specify={
                    'Heron': self.params['Heron'].value,
                    'TargetLocation': button_waypoint
                }
            ))

            # Operate door button
            if dom.type == 'scalable:DoorButton':
                # Detect door operating mechanism
                skill_list.append(self.skill('JPDetectDOM', 'jp_detect_dom', specify={
                    'Arm': arm,
                    'Mechanism': dom
                }))
                # press button
                skill_list.append(self.skill('ButtonPress', 'button_press', specify={
                    'Arm': arm,
                    'Button': dom
                }))
            elif dom.type == 'scalable:DoorHandle':
                # butt = dom
                # handle door handle
                # skill_list.append(self.skill('OperateHandle', 'operate_handle', specify={
                #     'Arm': arm,
                #     'Handle': butt, # with great enthusiasm
                # }))
                skill_list.append(self.skill('SuccessSkill', 'success_skill', specify={
                    'msg': 'Door not opened, drive and pray.'
                }))
            else:
                raise RuntimeError('Cannot handle this type of door operating mechanism.')

            # Pass door
            if door.type == 'scalable:Door':
                skill_list.append(self.skill('JPPassDoor', 'jp_pass_door', specify={
                    'Heron': self.params['Heron'].value,
                    'Door': door
                }))
            elif door.type == 'scalable:Elevator':
                skill_list.append(self.skill('JPPassElevator', 'jp_pass_elevator', specify={
                    'Heron': self.params['Heron'].value,
                    'Elevator': door
                }))
            else:
                raise RuntimeError('Cannot handle this type of door.')

        return skill_list

    def plan_path(self):
        dest = self.params['DestinationRegion'].value

        q = Queue()
        q.put((None, None, None, self.params['SourceRegion'].value))

        visited = set()
        prev = dict()

        while not q.empty():
            prev_region, dom, door, region = q.get()
            if region.label in visited:
                continue

            visited.add(region.label)
            prev[region.label] = (prev_region, dom, door)

            if dest.label == region.label:
                break

            for next_dom, next_door, next_region in self.get_connections(region):
                q.put((region, next_dom, next_door, next_region))

        if dest.label not in visited:
            return False, []
        
        path = []
        region = dest
        while True:
            prev_region, dom, door = prev[region.label]

            if door is None:
                break

            path.append((dom, door, region))
            region = prev_region
        
        return True, list(reversed(path))

    def get_connections(self, region):
        # TODO: find out if there is some fancy way to query the world model to simplify this //JP
        # TODO: not happening :) //JP
        connections = []
        relations = region.getRelations(subj='-1', pred='scalable:hasDoor')

        for relation in relations:
            door = self.wmi.get_element(relation['dst'])
            if self.wmi.get_super_class(door.type) != 'scalable:RegionTransition':
                continue

            next_loc = None
            next_region = None
            # Get door from WM
            for door_relation in door.getRelations(subj='-1', pred='skiros:hasA'):
                loc = self.wmi.get_element(door_relation['dst'])
                if loc.type == 'scalable:RegionBB' or self.wmi.get_super_class(loc.type) != 'scalable:Location':
                    continue
                
                temp = loc.getRelations(subj=region.id, pred='skiros:contain', obj='-1')

                if len(temp) > 0:
                    continue

                for loc_relation in loc.getRelations(pred='skiros:contain', obj='-1'):
                    find_region = self.wmi.get_element(loc_relation['src'])
                    if find_region.type != 'scalable:Region' and self.wmi.get_super_class(find_region.type) != 'scalable:Region':
                        continue

                    next_region = find_region

                next_loc = loc

                break
            if next_loc is None:
                raise RuntimeError('No door waypoint :(')
            
            dom_loc = None
            next_dom = None
            # Get correct button from WM
            for door_relation in door.getRelations(subj='-1', pred='skiros:hasA'):
                dom = self.wmi.get_element(door_relation['dst'])
                if self.wmi.get_super_class(dom.type) != 'scalable:DoorOperatingMechanism':
                    continue

                temp = dom.getRelations(subj='-1', pred='skiros:hasA')
                if len(temp) == 0:
                    continue

                has_region = False
                # Check if button is in the correct region
                for dom_relation in temp:
                    sub = self.wmi.get_element(dom_relation['dst'])
                    if sub.type == 'scalable:RegionBB' or self.wmi.get_super_class(sub.type) != 'scalable:Location':
                        continue

                    tmp = sub.getRelations(subj=region.id, pred='skiros:contain', obj='-1')
                    if len(tmp) > 0:
                        dom_loc = sub
                        has_region = True
                        break

                if not has_region:
                    continue
                next_dom = dom
                break

            if next_dom is None:
                raise RuntimeError('No butt :(')
            
            connections.append(((dom_loc, dom), (next_loc, door), next_region))
        
        return connections
