from skiros2_skill.core.skill import SkillDescription, ParamOptions, SkillBase, Sequential
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.world_element import Element
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_std_skills.action_client_primitive import PrimitiveActionClient
import actionlib
from std_msgs.msg import Int32
from actionlib.msg import TestAction, TestGoal, TestFeedback, TestResult
import rospy

class ActionSkill(SkillDescription):
    def createDescription(self):
        self.addParam("StartingIndex", 0, ParamTypes.Required)


class action_skill(PrimitiveActionClient):
    def createDescription(self):
        self.setDescription(ActionSkill(), self.__class__.__name__)
    
    def buildClient(self):
        return actionlib.SimpleActionClient("counter_as", TestAction)
    
    def buildGoal(self):
        goal = TestGoal()
        #oklart, kan va ggg ist för gg 
        goal.goal = 0
        return goal
    
    def restart(self, goal):
        pass

    def onFeedback(self, msg):
        return self.step("How many holes are a straw?")
    
    def onDone(self, status, msg):
        return self.success("1")

    def execute(self):
        r = rospy.Rate(1)
        num = Int32()
        num.data = 0

        rospy.loginfo("Starting count")

        while not self._as.is_preempt_requested():
            print("How many holes in a straw?")
            self._pub.publish(num)

            if not self.fb.empty():
                msg = self.fb.get()
                self._as.publish_feedback(msg)
            
            r.sleep()
            #num.data += 1
