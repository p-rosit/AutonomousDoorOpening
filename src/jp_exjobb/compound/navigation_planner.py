from skiros2_skill.core.skill import SkillBase, SkillDescription, Sequential
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

from queue import Queue

class NavigateBuilding(SkillDescription):
    def createDescription(self):
        self.addParam('Heron', Element('cora:Robot'), ParamTypes.Required)
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

    def build_skill_list(self, path):
        skill_list = []

        for ii in path:
            (w1, b), (w2, d), sm = ii
            print((b, d, sm))

        for node in path:
            (w1, butt), (w2, door), region = node
            skill_list.append(self.skill('JPDrive', 'jp_drive', 
                specify={
                    'Heron': self.params['Heron'].value,
                    'TargetLocation': w1
                }
            ))
            skill_list.append(self.skill('GoThroughDoor', 'go_through_door',
                specify={
                    'Heron': self.params['Heron'].value,
                    'Button': butt,
                    'TargetLocation': w2
                }
            ))

        return skill_list

    def plan_path(self):
        dest = self.params['DestinationRegion'].value

        q = Queue()
        q.put((None, None, None, self.params['SourceRegion'].value))

        visited = set()
        prev = dict()

        while not q.empty():
            prev_region, button, door, region = q.get()
            # print(prev_region)
            if region.label in visited:
                continue
            
            # print(prev_region)

            visited.add(region.label)
            prev[region.label] = (prev_region, button, door)

            if dest.label == region.label:
                break

            for next_button, next_door, next_region in self.get_connections(region):
                q.put((region, next_button, next_door, next_region))

        if dest.label not in visited:
            return False, []
        
        path = []
        region = dest
        while True:
            prev_region, button, door = prev[region.label]

            if door is None:
                break

            path.append((button, door, region))
            region = prev_region
        
        return True, list(reversed(path))

    def get_connections(self, region):
        # TODO: find out if there is some fancy way to query the world model to simplify this //JP
        # TODO: not happening :) //JP
        connections = []
        relations = region.getRelations(subj='-1', pred='scalable:hasDoor')

        for relation in relations:
            door = self.wmi.get_element(relation['dst'])
            if door.type != 'scalable:RegionTransition':
                continue

            next_loc = None
            next_region = None
            # Get door from WM
            for door_relation in door.getRelations(subj='-1', pred='skiros:hasA'):
                loc = self.wmi.get_element(door_relation['dst'])
                if loc.type != 'scalable:Waypoint':
                    continue
                
                temp = loc.getRelations(subj=region.id, pred='skiros:contain', obj='-1')

                if len(temp) > 0:
                    continue

                for loc_relation in loc.getRelations(pred='skiros:contain', obj='-1'):
                    find_region = self.wmi.get_element(loc_relation['src'])
                    if find_region.type != 'scalable:Region':
                        continue

                    next_region = find_region

                next_loc = loc

                break
            if next_loc is None:
                raise RuntimeError(':(')
            
            butt_loc = None
            next_butt = None
            # Get correct button from WM
            for door_relation in door.getRelations(subj='-1', pred='skiros:hasA'):
                butt = self.wmi.get_element(door_relation['dst'])
                if butt.type != 'scalable:DoorButton':
                    continue

                temp = butt.getRelations(subj='-1', pred='skiros:hasA')
                if len(temp) == 0:
                    continue

                has_region = False
                # Check if button is in the correct region
                for butt_relation in temp:
                    buttrub = self.wmi.get_element(butt_relation['dst'])
                    if buttrub.type != 'scalable:Waypoint':
                        continue

                    tmp = buttrub.getRelations(subj=region.id, pred='skiros:contain', obj='-1')
                    if len(tmp) > 0:
                        butt_loc = buttrub
                        has_region = True
                        break

                if not has_region:
                    continue
                next_butt = butt
                break

            if next_butt is None:
                raise RuntimeError('No butt :(')
            
            connections.append(((butt_loc, butt), (next_loc, door), next_region))
        
        return connections
