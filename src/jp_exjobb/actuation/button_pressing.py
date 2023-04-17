from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential, ParallelFs
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

from std_msgs.msg import Empty
from geometry_msgs.msg import PoseStamped, WrenchStamped

import rospy
import numpy as np

class ButtonPress(SkillDescription):
    def createDescription(self):
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Required)
        self.addParam('Gripper', Element('scalable:WsgGripper'), ParamTypes.Inferred)
        self.addParam('Button', Element('sumo:Object'), ParamTypes.Required)
        self.addParam('Offset', 0.0, ParamTypes.Required)
        self.addParam('Force', 10.0, ParamTypes.Required)

        self.addPreCondition(self.getRelationCond('ArmHasGripper', 'skiros:hasA', 'Arm', 'Gripper', True))

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
        return True

    def run(self):
        ind = 0
        while not self.force_goal_met and ind < self.hz * self.time_limit:
            ind +=1
            self.rate.sleep()

        self.running = False

        if not self.force_goal_met:
            return False, 'Did not reach force goal, but might still have pressed'
        return True, 'might have pressed?'


class button_press(SkillBase):
    def createDescription(self):
        self.setDescription(ButtonPress(), self.__class__.__name__)
    
    def init(self, wmi, instanciator):
        self.pre_press_pose = None
        self.press_pose = None
        self.prepresssub = rospy.Subscriber('/ButtonPress/PrePress', PoseStamped, callback=self.set_pre_pose)
        self.presssub = rospy.Subscriber('/ButtonPress/Press', PoseStamped, callback=self.set_press_pose)
        self.pressreplypub = rospy.Publisher('/ButtonPress/Reply', Empty, queue_size=1)
        return super().init(wmi, instanciator)

    def set_pre_pose(self, msg):
        self.pre_press_pose = Element('skiros:TransformationPose')
        self.pre_press_pose.setData(':PoseStampedMsg', msg)
        self.pressreplypub.publish(Empty())

    def set_press_pose(self, msg):
        self.press_pose = Element('skiros:TransformationPose')
        self.press_pose.setData(':PoseStampedMsg', msg)
        self.pressreplypub.publish(Empty())

    def onStart(self):
        self.pre_press_pose = None
        self.press_pose = None
        return True

    def expand(self, skill):
        compliant = Element('scalable:ControllerState')
        compliant.setProperty('rdfs:label', 'compliant')

        gripper = self.params['Gripper'].value
        button = self.params['Button'].value

        gripper_offset = gripper.getProperty('skiros:SizeZ').value
        offset = self.params['Offset'].value
        pre_pose = button.getData(':PoseStampedMsg')
        press_pose = button.getData(':PoseStampedMsg')

        pre_pose.pose.position.z -= gripper_offset + 0.02
        press_pose.pose.position.z += offset - gripper_offset

        self.pre_press_pose = Element('skiros:TransformationPose')
        self.pre_press_pose.setData(':PoseStampedMsg', pre_pose)
        self.press_pose = Element('skiros:TransformationPose')
        self.press_pose.setData(':PoseStampedMsg', press_pose)

        self.setProcessor(Sequential())
        skill(
            self.skill('ForceSensingOn', 'force_sensing_on', specify={'Compliant': True}),
            self.skill('ButtonPress','generate_press_poses'),
            self.skill('JPMoveArm','jp_move_arm', specify={'Target': self.pre_press_pose, 'Mode': compliant}),
            self.skill(ParallelFs())(
                self.skill('JPMoveArm','jp_move_arm', specify={'Target': self.press_pose, 'Mode': compliant}),
                self.skill('WaitForForce', 'wait_for_force')
            ),
            self.skill('JPMoveArm','jp_move_arm', specify={'Target': self.pre_press_pose, 'Mode': compliant}) 
        )

class generate_press_poses(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(ButtonPress(), self.__class__.__name__)
    
    def onInit(self):
        self.hz = 20
        self.time_limit = 1.5
        self.rate = rospy.Rate(self.hz)
        
        self.prepresspub = rospy.Publisher('/ButtonPress/PrePress', PoseStamped, queue_size=1)
        self.presspub = rospy.Publisher('/ButtonPress/Press', PoseStamped, queue_size=1)
        self.pressreplysub = rospy.Subscriber('/ButtonPress/Reply', Empty, callback=self.replied)
        return True

    def replied(self, _):
        self.replies += 1

    def preStart(self):
        self.replies = 0
        return True

    def run(self):
        gripper = self.params['Gripper'].value
        button = self.params['Button'].value

        gripper_offset = gripper.getProperty('skiros:SizeZ').value
        offset = self.params['Offset'].value
        pre_pose = button.getData(':PoseStampedMsg')
        press_pose = button.getData(':PoseStampedMsg')

        pre_pose.pose.position.z -= gripper_offset + 0.02
        press_pose.pose.position.z += offset - gripper_offset
        
        self.prepresspub.publish(pre_pose)
        self.presspub.publish(press_pose)

        ind = 0
        while self.replies < 2 and ind < self.hz * self.time_limit:
            ind += 1
            self.rate.sleep()
        
        if self.replies < 2:
            return False, 'did not get poses for pressing'
        
        return True, 'READY TO PRESS'
