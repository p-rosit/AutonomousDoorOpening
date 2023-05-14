from skiros2_skill.core.skill import SkillBase, SkillDescription, Sequential, Loop
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes

class LoopTest(SkillDescription):
    def createDescription(self):
        self.addParam('Base', Element('sumo:Object'), ParamTypes.Required)
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Required)
        self.addParam('Radius', 0.7, ParamTypes.Required)
        self.addParam('Origin Max Radius', 0.8, ParamTypes.Optional)
        self.addParam('Origin Min Radius', 0.5, ParamTypes.Optional)
        self.addParam('Max Angle (deg)', 50.0, ParamTypes.Required)
        self.addParam('Angle Interval (deg)', 10.0, ParamTypes.Optional)

class loop_test(SkillBase):
    def createDescription(self):
        self.setDescription(LoopTest(), self.__class__.__name__)
    
    def expand(self, skill):
        self.setProcessor(Sequential())
        skill(
            self.skill(Loop())(
                self.skill('MoveArmOnSphere', 'move_arm_on_sphere', specify={
                    'Base': self.params['Base'].value,
                    'Arm': self.params['Arm'].value,
                    'Radius': self.params['Radius'].value,
                    'Origin Max Radius': self.params['Origin Max Radius'].value,
                    'Origin Min Radius': self.params['Origin Min Radius'].value,
                    'Max Angle (deg)': self.params['Max Angle (deg)'].value,
                    'Angle Interval (deg)': self.params['Angle Interval (deg)'].value
                })
            )
        )

class TT(SkillDescription):
    def createDescription(self):
        self.addParam('Hand', Element('sumo:Object'), ParamTypes.Required)
        self.addParam('Marker', Element('sumo:Object'), ParamTypes.Required)

class tt(SkillBase):
    def createDescription(self):
        self.setDescription(TT(), self.__class__.__name__)
    
    def expand(self, skill):
        self.setProcessor(Sequential())
        skill(
            self.skill('StartHandEyeCalibration', ''),
            self.skill('JPSaveHandEyeCalibData', '', specify={
                'Hand': self.params['Hand'].value,
                'Marker': self.params['Marker'].value
            })
        )