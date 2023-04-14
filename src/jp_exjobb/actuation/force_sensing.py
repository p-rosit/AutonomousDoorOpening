from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.primitive_thread import PrimitiveThreadBase

import rospy
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Bool, Int32, Float64

import numpy as np
import matplotlib.pyplot as plt

class ListenToWrench(SkillDescription):
    def createDescription(self):
        self.addParam('Time (s)', 5.0, ParamTypes.Required)
        self.addParam('Name', '', ParamTypes.Required)

class listen_to_wrench(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(ListenToWrench(), self.__class__.__name__)

    def onInit(self):
        self.running = False
        self.sub = rospy.Subscriber('/cartesian_compliance_controller/ft_sensor_wrench', WrenchStamped, callback=self.listen)
        self.rate = rospy.Rate(10)

    def preStart(self):
        self.running = True
        self.preempt_requested = False
        self.force = []
        self.force_coords = ([], [], [])
        self.torque = []
        self.torque_coords = ([], [], [])
        self.status = 'Listening to "/wrench" topic.'
        return True
    
    def onPreempt(self):
        self.preempt_requested = True
        return self.fail('Listening preempted.', -1)

    def run(self):
        path = '/home/duploproject/'
        # path = '/home/pontus/'

        time_limit = rospy.Duration(self.params['Time (s)'].value)
        start_time = rospy.Time.now()

        while rospy.Time.now() - start_time < time_limit:
            if self.preempt_requested:
                return False, ''
            self.rate.sleep()
        
        self.running = False

        force_mag = np.array(self.force)
        torque_mag = np.array(self.torque)

        fig, axs = plt.subplots(1)
        axs.plot(force_mag)
        axs.plot(torque_mag)
        axs.legend(['Force', 'Torque'])
        plt.savefig(path + self.params['Name'].value + '_wrench.png')
        plt.cla()

        fig, axs = plt.subplots(2)
        axs[0].plot(self.force_coords[0])
        axs[0].plot(self.force_coords[1])
        axs[0].plot(self.force_coords[2])
        axs[0].legend(['force x', 'force y', 'force z'])
        axs[1].plot(self.torque_coords[0])
        axs[1].plot(self.torque_coords[1])
        axs[1].plot(self.torque_coords[2])
        axs[1].legend(['torque x', 'torque y', 'torque z'])
        plt.savefig(path + self.params['Name'].value + '_coords.png')
        plt.cla()

        return True, 'Done listening to "/wrench" topic.'

    def listen(self, msg):
        if self.running:
            fx = msg.wrench.force.x
            fy = msg.wrench.force.y
            fz = msg.wrench.force.z
            tx = msg.wrench.torque.x
            ty = msg.wrench.torque.y
            tz = msg.wrench.torque.z

            force_mag = np.sqrt(fx ** 2 + fy ** 2 + fz ** 2)
            torque_mag = np.sqrt(tx ** 2 + ty ** 2 + tz ** 2)

            self.force.append(force_mag)
            self.torque.append(torque_mag)

            [force.append(coord) for force, coord in zip(self.force_coords, [fx, fy, fz])]
            [torque.append(coord) for torque, coord in zip(self.torque_coords, [tx, ty, tz])]


class ForceSensingOn(SkillDescription):
    def createDescription(self):
        self.addParam('Compliant', False, ParamTypes.Required)


class force_sensing_on(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(ForceSensingOn(), self.__class__.__name__)

    def onInit(self):
        self.hz = 10
        self.time_limit = 1
        self.rate = rospy.Rate(self.hz)
        self.pub = rospy.Publisher('/cartesian_compliance_controller/switch_state', Bool, queue_size=1)
        self.sub = rospy.Subscriber('/cartesian_compliance_controller/state_reply', Bool, callback=self.reply_callback)
    
    def preStart(self):
        self.reply = False
        self.state_changed = False
        self.running = True
        return True

    def reply_callback(self, msg):
        if self.running:
            self.reply = True
            self.state_changed = msg.data

    def run(self):
        msg = Bool()
        msg.data = self.params['Compliant'].value        
        self.pub.publish(msg)

        count = 0
        while not self.reply and count < self.time_limit * self.hz:
            self.rate.sleep()
            count += 1

        self.running = False

        if self.reply:
            if self.state_changed:
                return True, 'Changed state.'
            else:
                return True, 'Already in desired state'
        
        return False, 'No reply from wrench.'

class ForceZero(SkillDescription):
    def createDescription(self):
        self.addParam('Adjust', False, ParamTypes.Required)

class adjust_force(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(ForceZero(), self.__class__.__name__)

    def onInit(self):
        self.hz = 10
        self.time_limit = 1.5
        self.rate = rospy.Rate(self.hz)
        self.pub = rospy.Publisher('/cartesian_compliance_controller/adjust_force', Bool, queue_size=1)
        self.sub = rospy.Subscriber('/cartesian_compliance_controller/force_reply', Bool, callback=self.reply_callback)
    
    def preStart(self):
        self.reply = False
        self.force_adjusted = False
        self.running = True
        return True

    def reply_callback(self, msg):
        if self.running:
            self.reply = True
            self.force_adjusted = msg.data

    def run(self):
        adjust_force = self.params['Adjust'].value
        msg = Bool()
        msg.data = adjust_force
        self.pub.publish(msg)

        count = 0
        while not self.reply and count < self.time_limit * self.hz:
            self.rate.sleep()
            count += 1

        self.running = False

        if self.reply:
            if adjust_force:
                if self.force_adjusted:
                    return True, 'Force adjusted.'
                else:
                    return False, 'Too much movement in signal.'
            else:
                if self.force_adjusted:
                    return True, 'Force offset removed.'
                else:
                    return True, 'No force offset to remove.'
        
        return False, 'No reply from force_zero.'

class EnableSmoothing(SkillDescription):
    def createDescription(self):
        self.addParam('Smooth signal', True, ParamTypes.Required)

class enable_smoothing(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(EnableSmoothing(), self.__class__.__name__)

    def onInit(self):
        self.hz = 10
        self.time_limit = 1.5
        self.rate = rospy.Rate(self.hz)
        self.pub = rospy.Publisher('/cartesian_compliance_controller/filter', Bool, queue_size=1)
        self.sub = rospy.Subscriber('/cartesian_compliance_controller/filter_reply', Bool, callback=self.reply_callback)
    
    def preStart(self):
        self.reply = False
        self.signal_smoothed = False
        self.running = True
        return True

    def reply_callback(self, msg):
        if self.running:
            self.reply = True
            self.signal_smoothed = msg.data

    def run(self):
        smooth = self.params['Smooth signal'].value
        msg = Bool()
        msg.data = smooth
        self.pub.publish(msg)

        count = 0
        while not self.reply and count < self.time_limit * self.hz:
            self.rate.sleep()
            count += 1

        self.running = False

        if self.reply:
            if smooth:
                if self.signal_smoothed:
                    return True, 'Signal smoothed.'
                else:
                    return True, 'Signal already being smoothed.'
            else:
                if self.signal_smoothed:
                    return True, 'Smoothing removed.'
                else:
                    return True, 'Smoothing already removed.'
        
        return False, 'No reply from smooth signal.'
    
class ExpSmooth(SkillDescription):
    def createDescription(self):
        self.addParam('Exponential smoothing', False, ParamTypes.Required)
    
class exp_smooth(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(ExpSmooth(), self.__class__.__name__)

    def onInit(self):
        self.hz = 10
        self.time_limit = 1.5
        self.rate = rospy.Rate(self.hz)
        self.pub = rospy.Publisher('/cartesian_compliance_controller/exp_avg', Bool, queue_size=1)
        self.sub = rospy.Subscriber('/cartesian_compliance_controller/exp_avg_reply', Bool, callback=self.reply_callback)
    
    def preStart(self):
        self.reply = False
        self.exp_smoothing = False
        self.running = True
        return True

    def reply_callback(self, msg):
        if self.running:
            self.reply = True
            self.exp_smoothing = msg.data

    def run(self):
        smooth = self.params['Exponential smoothing'].value
        msg = Bool()
        msg.data = smooth
        self.pub.publish(msg)

        count = 0
        while not self.reply and count < self.time_limit * self.hz:
            self.rate.sleep()
            count += 1

        self.running = False

        if self.reply:
            if smooth:
                if self.exp_smoothing:
                    return True, 'Exponential smoothing enabled.'
                else:
                    return True, 'Exponential smoothing already enabled.'
            else:
                if self.exp_smoothing:
                    return True, 'Moving average enabled.'
                else:
                    return True, 'Moving average already enabled.'
        
        return False, 'No reply from exp smooth.'
    
class WeightUpdate(SkillDescription):
    def createDescription(self):
        self.addParam('Last weight', 0.1, ParamTypes.Required)
    
class weight_update(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(WeightUpdate(), self.__class__.__name__)

    def onInit(self):
        self.hz = 10
        self.time_limit = 1.5
        self.rate = rospy.Rate(self.hz)
        self.pub = rospy.Publisher('/cartesian_compliance_controller/weight_update', Float64, queue_size=1)
        self.sub = rospy.Subscriber('/cartesian_compliance_controller/weight_reply', Bool, callback=self.reply_callback)
    
    def preStart(self):
        self.reply = False
        self.result = False
        self.running = True
        return True

    def reply_callback(self, msg):
        if self.running:
            self.reply = True
            self.result = msg.data

    def run(self):
        last_weight = self.params['Last weight'].value
        msg = Float64()
        msg.data = last_weight
        self.pub.publish(msg)

        count = 0
        while not self.reply and count < self.time_limit * self.hz:
            self.rate.sleep()
            count += 1

        self.running = False

        if self.reply:
            if self.result:
                return True, 'Last weight value set.'
            else:
                return False, 'Invalid weight value.'
        
        return False, 'No reply from weight update.'

class SizeUpdate(SkillDescription):
    def createDescription(self):
        self.addParam('New size', 20, ParamTypes.Required)

class size_update(PrimitiveThreadBase):
    def createDescription(self):
        self.setDescription(SizeUpdate(), self.__class__.__name__)

    def onInit(self):
        self.hz = 10
        self.time_limit = 1.5
        self.rate = rospy.Rate(self.hz)
        self.pub = rospy.Publisher('/cartesian_compliance_controller/size_update', Int32, queue_size=1)
        self.sub = rospy.Subscriber('/cartesian_compliance_controller/size_reply', Bool, callback=self.reply_callback)
    
    def preStart(self):
        self.reply = False
        self.result = False
        self.running = True
        return True

    def reply_callback(self, msg):
        if self.running:
            self.reply = True
            self.result = msg.data

    def run(self):
        mean_size = self.params['New size'].value
        msg = Int32()
        msg.data = mean_size
        self.pub.publish(msg)

        count = 0
        while not self.reply and count < self.time_limit * self.hz:
            self.rate.sleep()
            count += 1

        self.running = False

        if self.reply:
            if self.result:
                return True, 'Size of smoothing window updated.'
            else:
                return False, 'Size of smoothing window needs to be positive.'
        
        return False, 'No reply from size update.'