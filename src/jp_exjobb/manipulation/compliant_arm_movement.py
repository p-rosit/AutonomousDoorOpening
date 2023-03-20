from skiros2_skill.core.skill import SkillBase, Sequential

from skiros2_std_skills.action_client_primitive import PrimitiveActionClient
from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from jp_exjobb.manipulation.joint_arm_movement import JPMoveArm

import rospy
import actionlib
import tf2_ros

from cartesian_trajectory_generator.msg import TrajectoryAction, TrajectoryGoal
from copy import deepcopy
from actionlib_msgs.msg import GoalStatus

class jp_compliant_move_arm(SkillBase):
    def createDescription(self):
        self.setDescription(JPMoveArm(), self.__class__.__name__)
    
    def expand(self, skill):
        skill.setProcessor(Sequential())
        self.skill(
            self.skill('SwitchController', 'switch_controller', specify={'Controller': 'compliant'}),
            self.skill('JPMoveArm', 'jp_compliant_arm_movement')
        )

class jp_compliant_arm_movement(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(JPMoveArm(), self.__class__.__name__)
    
    def onInit(self):
        pass
    
    def onPreempt(self):
        self.skill_preempted = True
        return self.fail('Motion preempted', -1)

    def onPreStart(self):
        self.succeeded = False
    
    def run(self):

        # movement

        super().run()

    def onRunning(self):
        return self.step('Moving.')
    
    def onComplete(self):
        pass
    
    def onEnd(self):
        pass

class go_to_linear_JP(PrimitiveActionClient):
    """
    @brief Move arm directly to target (aligns x y z)
    """

    def createDescription(self):
        self.setDescription(JPMoveArm(), self.__class__.__name__)

    def buildClient(self):
        ns = ""
        # if self.params["Arm"].value.hasProperty("scalable:ArmId"):
        #     ns = self.params["Arm"].value.getProperty("scalable:ArmId").value
        return actionlib.SimpleActionClient("/" + ns + "/cartesian_trajectory_generator/goal_action",
                                            TrajectoryAction)

    def transform_to_frame(self, element, target_frame):
        if not element.hasProperty("skiros:FrameId", not_none=True):
            raise Exception("Missing pose info for target.")
        reasoner = element._getReasoner("AauSpatialReasoner")
        reasoner.get_transform(element.getProperty("skiros:FrameId").value, target_frame)
        reasoner.transform(element, target_frame)
        return element

    def buildGoal(self):
        target = self.params["Target"].value
        goal = TrajectoryGoal()
        if not target.hasProperty("skiros:FrameId", not_none=True):
            raise Exception("Missing frame_id of goal")
        parent_frame = target.getProperty("skiros:BaseFrameId").value
        target_msg = self.transform_to_frame(
            deepcopy(target), parent_frame).getData(":PoseStampedMsg")
        goal.header.frame_id = target.getProperty("skiros:BaseFrameId").value
        goal.header.stamp = rospy.Time.now()
        goal.goal = target_msg
        return goal

    def onFeedback(self, msg):
        # print('========')
        # print(msg)
        # print('========')
        # return self.step("Progress: {}%.".format(
        #     round(100 * msg.time_percentage), msg.trans_goal_error, msg.rot_goal_error)2)
        return self.step('69')

    def onDone(self, status, msg):
        if status == GoalStatus.ABORTED:
            return self.fail("Failed aborted", -2)
        elif status == GoalStatus.SUCCEEDED:
            return self.success("Succeeded")
        elif status == GoalStatus.REJECTED:
            rospy.loginfo(status)
            rospy.loginfo(msg)
            return self.fail("Goal was rejected by action server.", -2)
        else:
            rospy.loginfo(status)
            rospy.loginfo(msg)
            return self.fail("Unknown return code.", -100)