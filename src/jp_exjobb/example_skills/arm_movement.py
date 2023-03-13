import sys
import threading

from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential
from skiros2_std_skills.action_client_primitive import PrimitiveActionClient
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase
import moveit_commander

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
import tf2_ros

class JPMoveArm(SkillDescription):
    def createDescription(self):
        # self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Required)
        self.addParam("Arm", Element("rparts:ArmDevice"), ParamTypes.Required) # absolut shite
        self.addParam("Target", Element("sumo:Object"), ParamTypes.Required)
        # self.addParam("Target", Element("skiros:Product"), ParamTypes.Required)

class jp_move_arm(PrimitiveBase):
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
        return self.fail('Motion preempted.', -1)

    def onStart(self):
        self.complete = False
        self.succeeded = False
        self.preempt_motion = False

        arm = self.params['Arm'].value

        self.group = moveit_commander.move_group.MoveGroupCommander(arm.getProperty("skiros:MoveItGroup").value)
        self.group.set_pose_reference_frame(arm.getProperty("skiros:MoveItReferenceFrame").value)
        self.group.set_end_effector_link(arm.getProperty("skiros:MoveItTCPLink").value)
        self.group.set_max_velocity_scaling_factor(0.1)
        self.group.set_max_acceleration_scaling_factor(0.1)

        self.thread = threading.Thread(target=self.run)
        self.thread.start()

        return True

    def run(self):
        #do the flop
        arm = self.params['Arm'].value
        arm_frame = arm.getProperty("skiros:MoveItReferenceFrame").value

        obj = self.params['Target'].value
        goal = obj.getData(':PoseStampedMsg')
        goal.header.stamp = rospy.Time(0)
        
        goal = self.buffer.transform(goal, arm_frame, rospy.Duration(1))

        if self.group.go(goal, wait=True):
            self.succeeded = True

        self.complete = True

    def execute(self):
        if not self.complete or self.preempt_motion:
            return self.step('Running.')
        else:
            if self.succeeded:
                return self.success('Goal reached.')
            else:
                return self.fail('Movement failed.', -1)
    
    def onEnd(self):
        self.group.stop()
        self.group.clear_pose_targets()








class jp_move_arm_joint(PrimitiveBase):
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
        return self.fail('Stopping motion.', -1)

    def onStart(self):
        self.complete = False
        self.succeeded = False
        self.preempt_motion = False

        self.group = moveit_commander.move_group.MoveGroupCommander(self.params["Arm"].value.getProperty("skiros:MoveItGroup").value)
        self.group.set_pose_reference_frame(self.params["Arm"].value.getProperty("skiros:MoveItReferenceFrame").value)
        self.group.set_end_effector_link(self.params["Arm"].value.getProperty("skiros:MoveItTCPLink").value)
        self.group.set_max_velocity_scaling_factor(0.1)
        self.group.set_max_acceleration_scaling_factor(0.1)

        self.thread = threading.Thread(target=self.run)
        self.thread.start()

        return True

    def run(self):
        rospy.loginfo('Executing goto action.')

        # self.joint_goal = [3.1415, -1.5708, -1.5708, -1.5708, 1.5708, 2.0944]
        self.joint_goal = [
            1.5704865455627441,
            0.0031444269367675304,
            -2.8347482681274414,
            -0.32037051141772466,
            1.570810317993164,
            1.570784568786621
        ]
        # goal = self.params['Target'].value.getData(':PoseStampedMsg')
        # goal.header.stamp = rospy.Time(0)
        # goal.header.
        # print(goal)
        # arm_goal = self.buffer.transform(goal, self.params["Arm"].value.getProperty("skiros:MoveItReferenceFrame").value, rospy.Duration(1))
        # self.group.set_pose_target(arm_goal.pose)

        # print(arm_goal)

        if self.group.go(self.joint_goal, wait=True):
            self.succeeded = True

        self.complete = True

    def execute(self):
        if not self.complete or self.preempt_motion:
            return self.step('Running.')
        else:
            self.group.stop()
            self.group.clear_pose_targets()
            if self.succeded:
                return self.success('Goal reached.')
            else:
                return self.fail('Movement failed.', -1)

    def onEnd(self):
        return True

