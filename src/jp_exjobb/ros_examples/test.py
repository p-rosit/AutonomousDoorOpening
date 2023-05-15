from skiros2_common.core.primitive import PrimitiveBase
from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

class Test1(SkillDescription):
    def createDescription(self):
        self.addParam('Name', True, ParamTypes.Required)
        self.addParam('Camera', Element('skiros:DepthCamera'), ParamTypes.Inferred)

        self.addParam('named_output', Element('skiros:Parameter'), ParamTypes.SharedOutput)
        self.addParam('Linked', Element('skiros:DepthCamera'), ParamTypes.LinkedOutput)

class Test2(SkillDescription):
    def createDescription(self):
        self.addParam(('test1_skill', 'named_output'), Element('skiros:Parameter'), ParamTypes.SharedInput, remap='name')
        self.addParam(('test1_skill', 'Linked'), Element('skiros:DepthCamera'), ParamTypes.LinkedInput, remap='camera')

class test1_skill(PrimitiveBase):
    def createDescription(self):
        self.setDescription(Test1(), self.__class__.__name__)
    
    def execute(self):
        out = self.params['named_output'].value
        out.setProperty('scalable:PositionKnown', self.params['Name'].value)

        self.setOutput('named_output', out)
        
        camera = self.params['Camera'].value
        self.setOutput('Linked', camera)

        return self.success('1')

class test2_skill(PrimitiveBase):
    def createDescription(self):
        self.setDescription(Test2(), self.__class__.__name__)
    
    def execute(self):
        print(self.params['name'].value.getProperty('scalable:PositionKnown').value)
        print(self.params['camera'].value.getData(':Pose'))
        return self.success('2')

class TTEST(SkillDescription):
    def createDescription(self):
        self.addParam('Name', True, ParamTypes.Required)

class ttest(SkillBase):
    def createDescription(self):
        self.setDescription(TTEST(), self.__class__.__name__)
    
    def expand(self, skill):
        self.setProcessor(Sequential())
        skill(
            self.skill('Test1', 'test1_skill', specify={
                'Name': self.params['Name'].value
            }),
            self.skill('Test2', 'test2_skill')
        )