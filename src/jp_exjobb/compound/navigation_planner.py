from skiros2_skill.core.skill import SkillBase, SkillDescription, Sequential
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

from queue import Queue

class NavigateBuilding(SkillDescription):
    def createDescription(self):
        self.addParam('Heron', Element('cora:Robot'), ParamTypes.Required)
        self.addParam('Source', Element('scalable:Workstation'), ParamTypes.Inferred)
        self.addParam('Destination', Element('scalable:Workstation'), ParamTypes.Required)

        self.addParam('SourceRegion', Element('skiros:Location'), ParamTypes.Inferred)
        self.addParam('DestinationRegion', Element('skiros:Location'), ParamTypes.Inferred)

        self.addPreCondition(self.getRelationCond('HeronAtWorkstation', 'skiros:at', 'Heron', 'Source', True))
        self.addPreCondition(self.getRelationCond('SourceInRegion', 'skiros:contain', 'SourceRegion', 'Source', True))
        self.addPreCondition(self.getRelationCond('DestionationInRegion', 'skiros:contain', 'DestinationRegion', 'Destination', True))

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

        # skill(*skill_list)
        skill(self.skill('FailSkill', 'fail_skill', specify={'msg': 'Not Implemented'}))

    def plan_path(self):
        dest = self.params['DestinationRegion'].value

        q = Queue()
        q.put((None, None, None, self.params['SourceRegion'].value))

        visited = set()
        prev = dict()

        while not q.empty():
            prev_region, button, door, region = q.get()

            if region.label in visited:
                continue

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
            path.append((button, door, region))
            region = prev_region

            if door is None:
                break
        
        return True, reversed(path)

    def get_connections(self, region):
        connections = []
        source_region = self.params['SourceRegion'].value
        relations = source_region.getRelations(subj='-1', pred='skiros:hasA')
        # relations = self.params['SourceRegion'].value.getRelations(subj='-1', pred='scalable:hasDoor')

        for relation in relations:
            door = self.wmi.get_element(relation['dst'])
            if door.type != 'scalable:LocationTransition':
                continue

            next_loc = None
            for door_relation in door.getRelations(pred='skiros:hasA', obj='-1'):
                loc = self.wmi.get_element(door_relation['dst'])

                if loc.type != 'skiros:Location' or loc.label == source_region.label:
                    continue

                next_loc = loc
            
            if next_loc is None:
                raise RuntimeError(':(')
            
            connections.append((_, door, next_loc))
        
        return connections
        
    def build_skill_list(self, temp):
        pass