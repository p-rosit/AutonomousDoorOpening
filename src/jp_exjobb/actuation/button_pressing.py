from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential, ParallelFs, ParallelFf
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Empty, Bool

import rospy
import numpy as np

class ButtonPress(SkillDescription):
    def createDescription(self):
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Required)
        self.addParam('Gripper', Element('scalable:WsgGripper'), ParamTypes.Inferred)
        self.addParam('Pose', Element('skiros:TransformationPose'), ParamTypes.Inferred)
        self.addParam('Compliant', Element('scalable:ControllerState'), ParamTypes.Inferred)
        self.addPreCondition(self.getPropCond('CompliantController', 'skiros:Value', 'Compliant', '=', 'compliant', True))

        self.addParam('Button', Element('scalable:DoorButton'), ParamTypes.Required)
        self.addParam('Offset', 0.05, ParamTypes.Required)
        self.addParam('Force', 60.0, ParamTypes.Required)

        self.addPreCondition(self.getRelationCond('ArmHasGripper', 'skiros:hasA', 'Arm', 'Gripper', True))
        self.addPreCondition(self.getRelationCond('GripperHasPose', 'skiros:contain', 'Gripper', 'Pose', True))

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
            return False, 'Did not reach force goal.'
        return True, 'Force goal met.'

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
            return False, 'Force goal not met.'
        
        return True, 'Force goal met.'

class button_press(SkillBase):
    def createDescription(self):
        self.setDescription(ButtonPress(), self.__class__.__name__)

    def expand(self, skill):
        self.setProcessor(Sequential())
        skill(
            self.skill('ForceSensingOn', 'force_sensing_on', specify={'Compliant': True}),
            self.skill('GeneratePressPose','generate_press_pose', specify={'Offset': -0.1}),
            self.skill('JPMoveArm','jp_move_arm', remap={'Target': 'Pose'}, specify={'Mode': self.params['Compliant'].value}),

            self.skill('GeneratePressPose','generate_press_pose', specify={'Offset': self.params['Offset'].value}),
            self.skill(ParallelFs())(
                self.skill('JPMoveArm','jp_move_arm', remap={'Target': 'Pose'}, specify={'Mode': self.params['Compliant'].value}),
                self.skill('WaitForForce', 'wait_for_force', specify={'Force': self.params['Force'].value})
            ),
            self.skill('ForceCheck', 'force_check'),
            
            self.skill('GeneratePressPose','generate_press_pose', specify={'Offset': -0.1}),
            self.skill('JPMoveArm','jp_move_arm', remap={'Target': 'Pose'}, specify={'Mode': self.params['Compliant'].value})
        )

class GeneratePressPose(SkillDescription):
    def createDescription(self):
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Required)
        self.addParam('Gripper', Element('scalable:WsgGripper'), ParamTypes.Inferred)
        self.addParam('Pose', Element('skiros:TransformationPose'), ParamTypes.Inferred)

        self.addParam('Button', Element('sumo:Object'), ParamTypes.Required)
        self.addParam('Offset', 0.0, ParamTypes.Required)

        self.addPreCondition(self.getRelationCond('ArmHasGripper', 'skiros:hasA', 'Arm', 'Gripper', True))
        self.addPreCondition(self.getRelationCond('GripperHasPose', 'skiros:contain', 'Gripper', 'Pose', True))

class generate_press_pose(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(GeneratePressPose(), self.__class__.__name__)
    
    def run(self):
        gripper = self.params['Gripper'].value
        button = self.params['Button'].value
        pose = self.params['Pose'].value    

        gripper_offset = gripper.getProperty('skiros:SizeZ').value
        offset = self.params['Offset'].value

        pose.setProperty('skiros:BaseFrameId', button.getProperty('skiros:FrameId').value)
        pose.setProperty('skiros:PositionX', 0.0)
        pose.setProperty('skiros:PositionY', 0.0)
        pose.setProperty('skiros:PositionZ', offset - gripper_offset)
        pose.setProperty('skiros:OrientationX', 0.0)
        pose.setProperty('skiros:OrientationY', 0.0)
        pose.setProperty('skiros:OrientationZ', 1.0)
        pose.setProperty('skiros:OrientationW', 0.0)
        self.wmi.update_element_properties(pose)

        return True, 'Pose generated.'
