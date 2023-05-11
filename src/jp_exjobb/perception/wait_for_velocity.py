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
        self.addParam('Position Threshold', 5e-4, ParamTypes.Optional)
        self.addParam('Orientation Threshold', 5e-4, ParamTypes.Optional)

class wait_for_velocity(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(WaitForVelocity(), self.__class__.__name__)
    
    def onInit(self):
        self.hz = 10
        self.rate = rospy.Rate(self.hz)
        return True

    def preStart(self):
        self.pose = self.get_pose()
        return True

    def run(self):
        time_limit = self.params['Time Limit (s)'].value
        pos_th = self.params['Position Threshold'].value
        quat_th = self.params['Orientation Threshold'].value

        ind = 0
        self.rate.sleep()
        while ind < self.hz * time_limit:
            ind += 1
            pos_vel, quat_vel = self.approx_velocity()

            if pos_vel != 0.0 and quat_vel != 0.0:
                self.status = 'Pos: %f, Quat: %f' % (pos_vel, quat_vel)

            if (pos_vel != 0.0 and quat_vel != 0.0 and
                pos_vel < pos_th and quat_vel < quat_th):
                return self.success('Velocity reached threshold.')
            
            self.rate.sleep()
        
        return self.fail('Time limit reached.', -1)

    def approx_velocity(self):
        prev_pos, prev_quat = self.pose
        pos, quat = self.get_pose()

        pos_vel = np.linalg.norm(pos - prev_pos) * self.hz

        quat_vel = (prev_quat * quat.inv()).as_quat()
        if quat_vel[-1] < 0:
            quat_vel = -quat_vel
        quat_vel[-1] -= 1
        quat_vel = np.linalg.norm(quat_vel) * self.hz

        self.pose = (pos, quat)

        return pos_vel, quat_vel

    def get_pose(self):
        thing = self.params['Thing'].value

        pos = np.array(thing.getData(':Position'))
        quat = np.array(thing.getData(':Orientation'))
        quat = rot.from_quat(quat)

        return pos, quat
        
