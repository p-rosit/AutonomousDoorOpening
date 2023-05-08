from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes

import rospy

class TimerSkill(SkillDescription):
    def createDescription(self):
        self.addParam('Max Time (s)', 1.0, ParamTypes.Required)
    
class timer_skill(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(TimerSkill(), self.__class__.__name__)
    
    def onInit(self):
        self.hz = 50
        self.rate = rospy.Rate(self.hz)
        return True

    def preStart(self):
        self.preempted = False
        return True

    def onPreempt(self):
        self.preempted = True
        return self.fail('Time preempted.', -1)

    def run(self):
        time_limit = self.params['Max Time (s)'].value

        ind = 0
        while ind < time_limit * self.hz and not self.preempted:
            self.status = 'Time timing %.2f out of %.2f seconds.' % (ind / self.hz, time_limit)
            ind += 1
            self.rate.sleep()
        
        return self.success('Timer timed %.2f seconds.' % time_limit)

