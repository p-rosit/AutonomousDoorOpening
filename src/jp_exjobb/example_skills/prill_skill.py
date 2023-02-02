from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.params import ParamTypes
import rospy
from std_msgs.msg import Int32

class PrillSkill(SkillDescription):
    def createDescription(self):
        self.addParam('Number', 0, ParamTypes.Required)

class prill_skill(PrimitiveBase):
    def __init__(self, *args, **kwargs):
        self.msg = Int32()
        self.pub = rospy.Publisher('/print_test', Int32, queue_size=1)
        super(prill_skill, self).__init__(*args, **kwargs)

    def createDescription(self):
        self.setDescription(PrillSkill(), self.__class__.__name__)

    def onInit(self):
        return True

    def onPreempt(self):
        return True

    def onStart(self):
        return True

    def execute(self):
        print('Tisdag 31/1-2023')

        self.msg.data = int(self.params['Number'].value)
        self.pub.publish(self.msg)

        return self.success('Done')

    def onEnd(self):
        return True
