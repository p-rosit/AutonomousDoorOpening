from skiros2_skill.core.skill import SkillBase, Sequential

# from jp_exjobb.example_skills.non_blocking_skill import NonBlockingBase
from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from jp_exjobb.manipulation.joint_arm_movement import JPMoveArm

import rospy
import tf2_ros


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


