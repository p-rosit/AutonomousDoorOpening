#!/usr/bin/env python3

from skiros2_skill.core.skill import SkillDescription, ParamOptions, SkillBase, Sequential
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_std_skills.action_client_primitive import PrimitiveActionClient
import skiros2_common.tools.logger as log
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseActionResult, MoveBaseGoal
from actionlib_msgs.msg import GoalStatusArray
import rospy
import threading

class Navigation:
    def __init__(self):
        #turn from running to green when ok
        """
        TASK:
            Set up all the variables etc. you need in order to perform navigation tasks.
        """
        self.goal = MoveBaseGoal()
        self.PENDING = 0
        self.ACTIVE = 1
        self.PREEMPTED = 2
        self.SUCCEEDED = 3
        self.ERROR = 4
        self.status = 0
        self.pg = None
        self.oldPose = None
        client_move = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.client = client_move 

        self.shouldCancel = False   
        
    def set_goal_position(self, pose):
        self.goal.target_pose.pose.position.x = pose.position.x
        self.goal.target_pose.pose.position.y = pose.position.y
        self.goal.target_pose.pose.position.z = pose.position.z

    def set_goal_orientation(self, pose):
        self.goal.target_pose.pose.orientation.x = pose.orientation.x
        self.goal.target_pose.pose.orientation.y = pose.orientation.y
        self.goal.target_pose.pose.orientation.z = pose.orientation.z
        self.goal.target_pose.pose.orientation.w = pose.orientation.w

    def feedback_callback(feedback):
        print('[Feedback] %s' % feedback)

    def drive_to(self, pose, max_tries=5, sleep_seconds_between_tries=5.0):
        """
        TASK:
            Implement this function.
            Given a ROS Pose message, this function should make the robot drive to that location.
        TASK (OPTIONAL):
            Make the robot retry if it fails.
            In this case you should make use of the max_tries variable.
            Between each attempt, you should wait a number of seconds specified by
            the sleep_seconds_between_tries variable.

        Parameters:
            pose: A ROS Pose message object.
            max_tries: An integer describing the maximum number of driving attempts.
            sleep_seconds_between_tries: A float describing how long to sleep between attempts (in seconds).
        Returns:
            A boolean representing success/fail.

        """
        
        # rospy.loginfo("[CLIENT] Waiting for /move_base server ...")
        self.client.wait_for_server()
        # rospy.loginfo("[CLIENT] /move_base server started")
        # print("before loop")

        if(self.driveTo(pose)):
             return True 
        else :
            for i in range(2):
                print("going to recovery location")
                if self.oldPose is not None:
                    self.driveTo(self.oldPose)
                print("retrying")
                if(self.driveTo(pose)):
                    return True
            return False
        

    def feedback(self, feedback):
        if(self.shouldCancel):
            self.client.cancel_goal()
            self.shouldCancel = False

        if self.oldPose is None or distance(pose, self.oldPose) > 5:
            print("updating recovery pose")
            self.oldPose = feedback.base_position.pose
        
        # print('[Feedback] %s' % feedback)



    def driveTo(self, pose, max_tries=5, sleep_seconds_between_tries=5.0):
        count = 0
        success = True
        self.goal.target_pose.pose = pose
        self.goal.target_pose.header.frame_id = "map"
        while max_tries >= count:
            
            rospy.loginfo("[test] sending goal")
            self.client.send_goal(self.goal, feedback_cb = self.feedback)
            rospy.loginfo("[test] goal sent")
            self.client.wait_for_result()
            self.status = self.client.get_state()
            rospy.loginfo("[test] %s" % self.status)
            count += count

            if(self.status == self.SUCCEEDED or self.status == self.PREEMPTED):
                return True
            rospy.loginfo("[test] trying again")
            
        # All attempts failed
        if count == 5:
            rospy.loginfo("[FAILURE] Maximum amount of attempts reached, set as Failure")
            success = False
        rospy.loginfo('[test] %s' % success)

        rospy.sleep(sleep_seconds_between_tries)

        return success

def distance(pose1, pose2):
    diffX = pose1.base_position.pose.position.x - pose2.base_position.pose.position.x
    diffY = pose1.base_position.pose.position.y - pose2.base_position.pose.position.y
    dis = sqrt(diffX ** 2 + diffy ** 2)
    return dis
        
class Drive(SkillDescription):   
    def createDescription(self):
        # =======Params=========
        self.addParam("TargetLocation", Element("scalable:Workstation"),ParamTypes.Required)   

class drive(PrimitiveBase):

    def createDescription(self):
        self.setDescription(Drive(), self.__class__.__name__)

    def modifyDescription(self, skill):
        pass

    def onPreempt(self):
        self.navigation.shouldCancel = True
        return self.fail('Canceled', -1)

    def run(self):
        navigation = Navigation()
        self.navigation = navigation
        

        target_location = self.params["TargetLocation"].value
        reasoner = target_location._getReasoner("AauSpatialReasoner")
        reasoner.transform(target_location, "map")
        pose = target_location.getData(":PoseMsg")

        self.result = navigation.drive_to(pose)
        self.done = True

    def onStart(self):
        self.done = False
        self.thread = threading.Thread(target=self.run)
        self.thread.start()
        return True

    def execute(self):
        if not self.done:
            return self.step('Running...')

        self.thread.join()

        if self.result:
            return self.success('Done')
        else:
            return self.fail('Failed', -1)

# if __name__ == '__main__':
#     driver = drive()
#     real_drive = Drive()
#     real_drive.createDescription()
#     driver.run()