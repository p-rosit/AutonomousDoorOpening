import sys
from copy import deepcopy
from skiros2_skill.core.skill import SkillDescription, SkillBase, Sequential
from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_std_skills.action_client_primitive import PrimitiveActionClient
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element

import rospy
import actionlib

from cartesian_trajectory_generator.msg import TrajectoryAction, TrajectoryGoal
from actionlib_msgs.msg import GoalStatus

import moveit_commander

class JPMoveArm(SkillDescription):
    def createDescription(self):
        self.addParam('Arm', Element('scalable:Ur5'), ParamTypes.Required)
        self.addParam('Target', Element('skiros:TransformationPose'), ParamTypes.Required)
        self.addParam('Mode', Element('scalable:ControllerState'), ParamTypes.Required)

class jp_move_arm(SkillBase):
    """
    Summary:
        Moves a robot arm with a specific choice of controller.

    Required Input:
        Arm: The arm to control.
        Target: The target pose or joint states.
        Mode: The controller which should be used.

    Behaviour:
        This skill should move the specified arm to a specified set of joint
        states or a specified pose.
    """
    def createDescription(self):
        self.setDescription(JPMoveArm(), self.__class__.__name__)
    
    def expand(self, skill):
        target = self.params['Target'].value
        controller = self.params['Mode'].value
        mode = controller.getProperty('skiros:Value').value

        self.setProcessor(Sequential())

        if mode == 'compliant':
            if target.hasProperty('skiros:DataType') and not target.hasProperty('skiros:DataType', value='compliant'):
                skill(self.skill('FailSkill', 'fail_skill', specify={'msg': 'Target is not a pose in the world.'}))
                return
        elif mode == 'joint_config':
            if not target.hasProperty('skiros:DataType', value='joint_values', not_none=True):
                skill(self.skill('FailSkill', 'fail_skill', specify={'msg': 'Target does not contain joint values.'}))
                return

        if mode == 'joint_config':
            skill(
                self.skill('JPSwitchController', 'jp_switch_controller', specify={'Controller': controller}),
                self.skill('JPMoveArm', 'jp_primitive_joint')
            )
        elif mode == 'compliant':
            skill(
                self.skill('JPSwitchController', 'jp_switch_controller', specify={'Controller': controller}),
                self.skill('JPMoveArm', 'jp_primitive_compliant')
            )
        else:
            skill(self.skill('FailSkill', 'fail_skill', specify={'msg': 'Unknown controller state.'}))

class jp_primitive_joint(PrimitiveThreadBase):
    """
    Summary:
        Moves a robot arm to a joint state with moveit.

    Required Input:
        Arm: The arm to control.
        Target: Target joint states.
        Mode: unused

    Behaviour:
        Moves a 6 DOF robot arm to a set of joint states with moveit. The joint
        states are assumed to be saved in a TransformationPose in the following way

            joint_value_1 = skiros:PositionX
            joint_value_2 = skiros:PositionY
            joint_value_3 = skiros:PositionZ
            joint_value_4 = skiros:OrientationX
            joint_value_5 = skiros:OrientationY
            joint_value_6 = skiros:OrientationZ

    Notes and Pitfalls:
        If this skill is given a pose instead of a set of joint states expect
        the arm to spazz out.
    """
    def createDescription(self):
        self.setDescription(JPMoveArm(), self.__class__.__name__)
    
    def onInit(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        return True
    
    def onPreempt(self):
        self.preempt_motion = True
        self.group.stop()
        return self.fail('Motion preempted.', -1)

    def preStart(self):
        self.preempt_motion = False

        arm = self.params['Arm'].value

        self.group = moveit_commander.move_group.MoveGroupCommander(arm.getProperty("skiros:MoveItGroup").value)
        self.group.set_planner_id("RRTConnect")
        self.group.allow_replanning(True)
        self.group.set_planning_time(20)
        
        self.group.set_pose_reference_frame(arm.getProperty("skiros:MoveItReferenceFrame").value)
        self.group.set_end_effector_link(arm.getProperty("skiros:MoveItTCPLink").value)
        self.group.set_max_velocity_scaling_factor(0.1)
        self.group.set_max_acceleration_scaling_factor(0.1)
        self.group.clear_pose_target(arm.getProperty("skiros:MoveItTCPLink").value)
        self.group.clear_pose_targets()

        return True

    def run(self):
        #do the flop
        joint_state = self.params['Target'].value

        joint_goal = [
            joint_state.getProperty('skiros:PositionX').value,
            joint_state.getProperty('skiros:PositionY').value,
            joint_state.getProperty('skiros:PositionZ').value,
            joint_state.getProperty('skiros:OrientationX').value,
            joint_state.getProperty('skiros:OrientationY').value,
            joint_state.getProperty('skiros:OrientationZ').value
        ]

        if self.group.go(joint_goal, wait=True):
            return self.success('Goal Reached')
        
        return self.fail('Movement Failed', -1)

    
    def onEnd(self):
        self.group.stop()
        self.group.clear_pose_target(self.params['Arm'].value.getProperty("skiros:MoveItTCPLink").value)
        self.group.clear_pose_targets()
        return True

class jp_primitive_compliant(PrimitiveActionClient):
    """
    Summary:
        Moves a robot arm with the cartesian compliant controller.

    Required Input:
        Arm: The arm to control.
        Target: Target joint states.
        Mode: unused

    Behaviour:
        Moves a robot arm along a linear path in Cartesian space. The input
        is expected to be a TransformationPose with a pose defined in any frame.
        The input TransformationPose is then transformed to the correct frame in
        the skill.

        A linear path from the current gripper pose to the target gripper
        pose is generated by the controller and then the gripper tries to follow
        that path, obstacles and self-collision are not taken into account.

        The Cartesian compliant controller tries to make sure that the force the
        gripper feels from the force-torque sensor is zero. I.e. the controller
        tries to move the gripper away from any force it feels.
 
    Notes and Pitfalls:
        If a set of joint states is given to this skill expect the controller
        to spazz out.

        The pose is not verified to be reachable, if a pose which is not reachable
        is specified expect the controller to spazz out.
    """
    def createDescription(self):
        self.setDescription(JPMoveArm(), self.__class__.__name__)

    def buildClient(self):
        return actionlib.SimpleActionClient("/cartesian_trajectory_generator/goal_action", TrajectoryAction)

    def buildGoal(self):
        target = self.params["Target"].value

        goal = TrajectoryGoal()
        
        goal.header.frame_id = deepcopy(target.getProperty("skiros:BaseFrameId").value)
        goal.header.stamp = rospy.Time.now()
        goal.goal = deepcopy(target.getData(":PoseStampedMsg"))
        return goal

    def onFeedback(self, msg):
        return self.step("Progress: {}%. Trans-error: {:.3f} Rot-error: {:.2f}".format(
            round(100 * msg.time_percentage), msg.trans_goal_error, msg.rot_goal_error))

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