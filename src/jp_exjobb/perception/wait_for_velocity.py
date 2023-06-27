from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.primitive_thread import PrimitiveThreadBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes
from skiros2_skill.core.skill import SkillDescription

import rospy
import numpy as np
from scipy.spatial.transform import Rotation as rot

class WaitForVelocity(SkillDescription):
    """
    Summary:
        Waits until an object has stopped moving.

    Required Input:
        Thing:          The thing which needs to be stationary
        Time Limit (s): The maximal time limit

    Optional input:
        Position Threshold:     The smallest magnitude of the velocity before
                                the thing is considered stationary.
        Orientation Threshold:  The smallest magnitude of the rotational velocity
                                before the thing is considered stationary.

    Behaviour:
        Continually checks the pose of the input object and succeeds when the
        object is determined to be stationary. If the object does not become
        stationary within the given time the skill succeeds anyway but notes
        that it was timed out in the shared output 'timed_out'.

    Notes and Pitfalls:
        Currently the estimate of the velocity and rotational velocity is not
        strictly correct, one should use the time between the previous and current
        pose as the delta time. Currently a fixed delta time is used.
    """
    def createDescription(self):
        self.addParam('Thing', Element('sumo:Object'), ParamTypes.Required)
        self.addParam('Time Limit (s)', 3.0, ParamTypes.Required)
        self.addParam('Position Threshold', 5e-4, ParamTypes.Optional)
        self.addParam('Orientation Threshold', 5e-4, ParamTypes.Optional)

        self.addParam('timed_out', Element('skiros:Parameter'), ParamTypes.SharedOutput)

class CheckWaitVelocity(SkillDescription):
    def createDescription(self):
        self.addParam('Fail on timeout', True, ParamTypes.Required)
        self.addParam(('wait_for_velocity', 'timed_out'), Element('skiros:Parameter'), ParamTypes.SharedInput)

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
        timed_out = self.params['timed_out'].value
        time_limit = self.params['Time Limit (s)'].value
        pos_th = self.params['Position Threshold'].value
        quat_th = self.params['Orientation Threshold'].value

        timed_out.setProperty('skiros:Value', False)
        self.setOutput('timed_out', timed_out)

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
        
        timed_out.setProperty('skiros:Value', True)
        self.setOutput('timed_out', timed_out)

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
        # TODO: extract current time from thing's stamped pose
        # to correctly measure velocity
        quat = rot.from_quat(quat)

        return pos, quat
        
class check_wait_velocity(PrimitiveBase):
    """
    Summary:
        Checks if the previous wait for velocity succeded.

    Behaviour:
        Fails if the previous wait for velocity timed out, succeeds otherwise.
    """
    def createDescription(self):
        self.setDescription(CheckWaitVelocity(), self.__class__.__name__)
    
    def execute(self):
        timed_out = self.params['velocity_timed_out'].value.getProperty('skiros:Value').value
        fail_on_timeout = self.params['Fail on timeout'].value

        if not timed_out:
            return self.success('Timer did not time out.')

        if not fail_on_timeout:
            return self.success('Timed out.')
        else:
            return self.fail('Timed out.', -1)
