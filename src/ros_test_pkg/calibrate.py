"""
from skiros2_skill.core.skill import SkillDescription
from skiros2_std_skills.action_client_primitive import PrimitiveActionClient

import rospy
import actionlib
from std_msgs.msg import Empty
from sensor_msgs.msg import Image

class Calibrate(SkillDescription):
	def createDescription(self):
		pass

class calibrate(PrimitiveActionClient):
	def createDescription(self):
		self.setDescription(Calibrate(), self.__class__.__name__)

	def onStart(self):
		return True

	def buildClient(self):
		pass

	def buildGoal(self):
		return Empty()

	def restart(self, goal, text="Restarting action"):
		return self.step(text)

"""
