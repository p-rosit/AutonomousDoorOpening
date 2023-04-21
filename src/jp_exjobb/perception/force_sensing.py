from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes

from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Empty, Bool

import rospy
import numpy as np

class WaitForForce(SkillDescription):
    def createDescription(self):
        self.addParam('Time limit', 10.0, ParamTypes.Required)
        self.addParam('Force', 10.0, ParamTypes.Required)
        
class wait_for_force(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(WaitForForce(), self.__class__.__name__)

    def onInit(self):
        self.running = False
        self.hz = 50
        self.rate = rospy.Rate(self.hz)
        self.forcesub = rospy.Subscriber('/cartesian_compliance_controller/ft_sensor_wrench', WrenchStamped, callback=self.earing)
        self.reset_pub = rospy.Publisher('/wait_for_force/reset', Empty, queue_size=1)
        self.status_pub = rospy.Publisher('/wait_for_force/status', Bool, queue_size=1)
    
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
        self.reset_pub.publish(Empty())

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

            print(self.force_limit)

        self.running = False
        self.status_pub.publish(Bool(self.force_goal_met))

        if not self.force_goal_met:
            return self.fail('Did not reach force goal.', -1)
        return self.success('Force goal met.')

class ForceCheck(SkillDescription):
    def createDescription(self):
        pass

class force_check(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(ForceCheck(), self.__class__.__name__)
    
    def onInit(self):
        self.hz = 20
        self.time_limit = 1
        self.rate = rospy.Rate(self.hz)

        self.reply_received = False
        self.force_goal_met = False
        self.reset_sub = rospy.Subscriber('/wait_for_force/reset', Empty, callback=self.reset_callback)
        self.status_sub = rospy.Subscriber('/wait_for_force/status', Bool, callback=self.status_callback)
        return True
    
    def status_callback(self, msg):
        self.reply_received = True
        self.force_goal_met = msg.data

    def reset_callback(self, _):
        self.reply_received = False
        self.force_goal_met = False

    def run(self):
        ind = 0
        while not self.reply_received and ind < self.time_limit * self.hz:
            ind += 1
            self.rate.sleep()
        
        if not self.force_goal_met:
            return self.fail('Force goal not met.', -1)
        
        return self.success('Force goal met.')
