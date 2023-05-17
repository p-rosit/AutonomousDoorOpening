from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Empty, Bool

import rospy
import numpy as np

class WaitForForce(SkillDescription):
    def createDescription(self):
        self.addParam('Time limit', 10.0, ParamTypes.Required)
        self.addParam('Force', 10.0, ParamTypes.Required)
        self.addParam('force_goal_met', Element('skiros:Parameter'), ParamTypes.SharedOutput)
        
class wait_for_force(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(WaitForForce(), self.__class__.__name__)

    def onInit(self):
        self.running = False
        self.hz = 50
        self.rate = rospy.Rate(self.hz)
        self.forcesub = rospy.Subscriber('/wrench_filter/filtered_wrench', WrenchStamped, callback=self.earing)
        return True
    
    def earing(self, msg):
        if self.running:
            force = msg.wrench.force
            mag = np.sqrt(force.x**2 + force.y**2 + force.z**2)
            
            if self.force_limit < mag:
                self.force_goal_met = True

    def preStart(self):
        self.force_limit = self.params['Force'].value
        self.time_limit = self.params['Time limit'].value
        self.force_goal_met = False
        self.running = True
        self.preempted = False
        return True
    
    def onPreempt(self):
        self.preempted = True
        return self.fail('Pushing preempted',-1)

    def run(self):
        force_goal_met = self.params['force_goal_met'].value

        ind = 0
        wait_period = True
        start_time=rospy.Time.now().to_sec()
        while not self.force_goal_met and ind < self.hz * self.time_limit and not self.preempted:
            ind += 1
            self.rate.sleep()
            if wait_period and ind > self.hz:
                wait_period = False
                force_time = rospy.Time.now().to_sec()
                k = -self.force_limit / (self.time_limit - (force_time - start_time))
                m = self.force_limit
            
            if not wait_period:
                self.force_limit = k * (rospy.Time.now().to_sec() - force_time) + m

        self.running = False
        self.status_pub.publish(Bool(self.force_goal_met))

        if not self.force_goal_met:
            force_goal_met.setProperty('skiros:Value', False)
            self.setOutput('force_goal_met', force_goal_met)
            return self.fail('Did not reach force goal.', -1)
        
        force_goal_met.setProperty('skiros:Value', True)
        self.setOutput('force_goal_met', force_goal_met)
        return self.success('Force goal met.')

class ForceCheck(SkillDescription):
    def createDescription(self):
        self.addParam(('wait_for_force', 'force_goal_met'), Element('skiros:Parameter'), ParamTypes.SharedInput)

class force_check(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(ForceCheck(), self.__class__.__name__)
    
    def run(self):
        force_goal_met = self.params['force_goal_met'].value
        goal_met = force_goal_met.getProperty('skiros:Value').value

        if not goal_met:
            return self.fail('Force goal not met.', -1)
        
        return self.success('Force goal met.')
