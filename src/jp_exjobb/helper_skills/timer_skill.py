from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

import rospy

class TimerSkill(SkillDescription):
    def createDescription(self):
        self.addParam('Max Time (s)', 1.0, ParamTypes.Required)
        self.addParam('timed_out', Element('skiros:Parameter'), ParamTypes.SharedOutput)

class CheckTimer(SkillDescription):
    def createDescription(self):
        self.addParam('Fail on timeout', True, ParamTypes.Required)
        self.addParam(('timer_skill', 'timed_out'), Element('skiros:Parameter'), ParamTypes.SharedInput)

class timer_skill(PrimitiveThreadBase):
    """
    Summary:
        A timer.

    Required Input:
        Max Time (s): A positive float, the maximal time the timer counts to.

    Behaviour:
        If the maximal time limit is reached the skill succeeds. A boolean is
        set which keeps track of if the timer timed out.

    Notes and Pitfalls:
        This skill can be used in parallel with, for example, the compliant
        controller, if the goal is not reached within the time limit the
        skill will progress anyway due to this time limit.
    """
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
        return self.success('Time preempted.')

    def run(self):
        timed_out = self.params['timed_out'].value
        time_limit = self.params['Max Time (s)'].value

        timed_out.setProperty('skiros:Value', False)
        self.setOutput('timed_out', timed_out)

        ind = 0
        while ind < time_limit * self.hz and not self.preempted:
            self.status = 'Time timing %.2f out of %.2f seconds.' % (ind / self.hz, time_limit)
            ind += 1
            self.rate.sleep()
        
        timed_out.setProperty('skiros:Value', True)
        self.setOutput('timed_out', timed_out)

        return self.success('Timer timed %.2f seconds.' % time_limit)

class check_timer(PrimitiveBase):
    """
    Summary:
        Checks if the timer timed out.

    Required Input:
        Fail on timeout:    A boolean which determines if the skill should
                            fail or not if the timer times out.

    Behaviour:
        If the previous timer timed out the skill will fail, or not.
        Depending on what the input is set to.
    """
    def createDescription(self):
        self.setDescription(CheckTimer(), self.__class__.__name__)
    
    def execute(self):
        timed_out = self.params['timed_out'].value.getProperty('skiros:Value').value
        fail_on_timeout = self.params['Fail on timeout'].value

        if not timed_out:
            return self.success('Timer did not time out.')

        if not fail_on_timeout:
            return self.success('Timed out.')
        else:
            return self.fail('Timed out.', -1)