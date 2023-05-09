from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes
from skiros2_skill.core.skill import SkillDescription

import rospy
import numpy as np
from scipy.spatial.transform import Rotation as rot

class WaitForVelocity(SkillDescription):
    def createDescription(self):
        self.addParam('Thing', Element('sumo:Object'), ParamTypes.Required)
        self.addParam('Time Limit (s)', 3.0, ParamTypes.Required)
        self.addParam('Position Threshold', 1e-3, ParamTypes.Optional)
        self.addParam('Orientation Threshold', 1e-3, ParamTypes.Optional)

class wait_for_velocity(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(WaitForVelocity(), self.__class__.__name__)
    
    def onInit(self):
        self.hz = 20
        self.rate = rospy.Rate(self.hz)
        return True

    def preStart(self):
        self.pose = self.get_pose()
        return True

    def run(self):
        time_limit = self.params['Time Limit (s)'].value

        ind = 0
        while ind < self.hz * time_limit:
            ind += 1
            vel = self.approx_velocity()
            print(vel)
            self.rate.sleep()
        
        return self.success('')

    def approx_velocity(self):
        prev_pos, prev_quat = self.pose
        pos, quat = self.get_pose()

        pos_vel = 0.0
        quat_vel = 0.0

        return pos_vel, quat_vel

    def get_pose(self):
        thing = self.params['Thing'].value

        pos = np.array(thing.getData(':Position'))
        quat = np.array(thing.getData(':Orientation'))
        quat = rot.from_quat(quat)

        return pos, quat
        
