import sys
import threading

from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase
import moveit_commander

import rospy

class JPArm(SkillDescription):
    def createDescription(self):
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Required)

class jp_arm_home(PrimitiveBase):
    def createDescription(self):
        self.setDescription(JPArm(), self.__class__.__name__)
        
    def onInit(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
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
        rospy.loginfo('Executing goto action.')

        self.joint_goal = [
            1.5704865455627441,
            0.0031444269367675304,
            -2.8347482681274414,
            -0.32037051141772466,
            1.570810317993164,
            1.570784568786621
        ]
        if self.group.go(self.joint_goal, wait=True):
            self.succeeded = True

        self.complete = True

    def execute(self):
        if not self.complete or self.preempt_motion:
            return self.step('Running.')
        else:
            self.thread.join()
            if self.succeeded:
                return self.success('Home reached.')
            else:
                return self.fail('Movement failed.', -1)

    def onEnd(self):
        self.group.stop()
        self.group.clear_pose_target(self.params['Arm'].value.getProperty("skiros:MoveItTCPLink").value)
        self.group.clear_pose_targets()
        return True

