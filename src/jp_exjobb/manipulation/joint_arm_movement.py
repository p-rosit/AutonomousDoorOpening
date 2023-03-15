import sys
import threading
from copy import deepcopy

from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase
import moveit_commander

# from jp_exjobb.example_skills.non_blocking_skill import NonBlockingBase

import rospy
import tf2_ros

class JPMoveArm(SkillDescription):
    def createDescription(self):
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Required)
        # self.addParam("Arm", Element("rparts:ArmDevice"), ParamTypes.Required) # absolut shite
        self.addParam("Target", Element("sumo:Object"), ParamTypes.Required)
        # self.addParam("Target", Element("skiros:Product"), ParamTypes.Required)

class jp_move_arm(SkillBase):
    def createDescription(self):
        self.setDescription(JPMoveArm(), self.__class__.__name__)

    def expand(self, skill):
        skill.setProcessor(Sequential())
        skill(
            self.skill('SwitchController', 'switch_controller', specify={'Controller': 'joint_config'}),
            self.skill('JPMoveArm', 'jp_arm_movement')
        )

class jp_arm_movement(NonBlocking):
    def createDescription(self):
        self.setDescription(JPMoveArm(), self.__class__.__name__)
    
    def onInit(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.buffer = tf2_ros.Buffer()  # type: any
        self.tf_listener = tf2_ros.TransformListener(self.buffer)
        return True
    
    def onPreempt(self):
        self.preempt_motion = True
        self.group.stop()
        return self.fail('Motion preempted.', -1)

    def onStart(self):
        self.complete = False
        self.succeeded = False
        self.preempt_motion = False

        arm = self.params['Arm'].value

        self.group = moveit_commander.move_group.MoveGroupCommander(arm.getProperty("skiros:MoveItGroup").value)
        self.group.set_planner_id("RRTConnect")
        self.group.allow_replanning(True)
        
        self.group.set_pose_reference_frame(arm.getProperty("skiros:MoveItReferenceFrame").value)
        self.group.set_end_effector_link(arm.getProperty("skiros:MoveItTCPLink").value)
        self.group.set_max_velocity_scaling_factor(0.1)
        self.group.set_max_acceleration_scaling_factor(0.1)
        self.group.clear_pose_target(arm.getProperty("skiros:MoveItTCPLink").value)
        self.group.clear_pose_targets()

        self.thread = threading.Thread(target=self.run)
        self.thread.start()

        return True

    def run(self):
        #do the flop
        arm = self.params['Arm'].value
        arm_frame = arm.getProperty("skiros:MoveItReferenceFrame").value

        obj = self.params['Target'].value
        goal = deepcopy(obj.getData(':PoseStampedMsg'))
        goal.header.stamp = rospy.Time(0)
        
        goal = self.buffer.transform(goal, arm_frame, rospy.Duration(1))

        if self.group.go(goal.pose, wait=True):
            self.succeeded = True

        self.complete = True

    def execute(self):
        if not self.complete or self.preempt_motion:
            return self.step('Running.')
        else:
            self.thread.join()
            if self.succeeded:
                return self.success('Goal reached.')
            else:
                return self.fail('Movement failed.', -1)
    
    def onEnd(self):
        self.group.stop()
        self.group.clear_pose_target(self.params['Arm'].value.getProperty("skiros:MoveItTCPLink").value)
        self.group.clear_pose_targets()
        return True
