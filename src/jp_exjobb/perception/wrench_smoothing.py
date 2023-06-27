from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.primitive_thread import PrimitiveThreadBase

import rospy
from std_msgs.msg import Empty, Bool, Int32, Float64

"""
Note that all the skills in this script send and recieve messages to the node
/wrench_relay which performs the necessary filtering of the force. This node
can be found under

    robots/heron_robot/heron_control/src/wrench_intercept.py
"""

class ForceSensingOn(SkillDescription):
    def createDescription(self):
        self.addParam('Compliant', False, ParamTypes.Required)

class force_sensing_on(PrimitiveThreadBase):
    """
    Summary:
        Turns on the force sensing for Heron.

    Required Input:
        Compliant: A boolean denoting if the force sensing should be on or off.

    Behaviour:
        Publishes a boolean which decides if the force sensing should be on or off.
        Expect the wrench relay to reply to make sure that the message was recieved
        and handled correctly.
    """
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
                return self.success('Changed state.')
            else:
                return self.success('Already in desired state')
        
        return self.fail('No reply from wrench.', -1)

class AdjustForce(SkillDescription):
    def createDescription(self):
        self.addParam('Adjust', False, ParamTypes.Required)

class adjust_force(PrimitiveThreadBase):
    """
    Summary:
        Adjusts the force sensing by offseting the estimated force and torque.

    Required Input:
        Adjust: A boolean denoting if the force should be adjusted or if the
                previous adjustement should be removed.

    Behaviour:
        Publishes a boolean which decides if the force should be adjusted or if
        the previous adjustement should be removed.

        If the boolean is true the wrench relay does the following:

            - Records the current force for 5 seconds.
            - Computes the average of the force during the recorded time.
            - For any measurements after this the estimated mean is subtracted
              from the measured force.
        
        If the boolean is false the wrench relay does the following:

            - Removes the previously estimated force offset.
            - No offset is then used for future measurements.
    
    Notes and Pitfalls:
        This skill is meant to be used to remove any bias in the force-torque
        sensor. The gripper should be stationary when using the skill which ensures
        that the force the gripper experiences at its current pose is estimated
        correctly. I.e. we expect the current force the gripper is feeling to
        be zero.
    """
    def createDescription(self):
        self.setDescription(AdjustForce(), self.__class__.__name__)

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
                    return self.success('Force adjusted.')
                else:
                    return self.fail('Too much movement in signal.', -1)
            else:
                if self.force_adjusted:
                    return self.success('Force offset removed.')
                else:
                    return self.success('No force offset to remove.')
        
        return self.fail('No reply from force_zero.', -1)

class EnableSmoothing(SkillDescription):
    def createDescription(self):
        self.addParam('Smooth signal', True, ParamTypes.Required)

class enable_smoothing(PrimitiveThreadBase):
    """
    Summary:
        Enables or disables smoothing.

    Required Input:
        Smooth signal: A boolean denoting if smoothing should be on or off.

    Behaviour:
        A boolean is published which tells the wrench relay whether the
        force-torque should be smoothed or not. By default the last 20
        measurements are used to compute the filtered result.

    Notes and Pitfalls:
        The amount of measurements used to smooth the signal can be
        changed with the size_update skill.
    """
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
                    return self.success('Signal smoothed.')
                else:
                    return self.success('Signal already being smoothed.')
            else:
                if self.signal_smoothed:
                    return self.success('Smoothing removed.')
                else:
                    return self.success('Smoothing already removed.')
        
        return self.fail('No reply from smooth signal.', -1)
    
class ExpSmooth(SkillDescription):
    def createDescription(self):
        self.addParam('Exponential smoothing', False, ParamTypes.Required)
    
class exp_smooth(PrimitiveThreadBase):
    """
    Summary:
        Enables or disables exponential smoothing.

    Required Input:
        Exponential smoothing:  A boolean denoting if exponential smoothing
                                should be on or off.

    Behaviour:
        A boolean is published which tells the wrench relay whether the
        smoothing should be exponential or a moving average.

        The smoothing is not turned on or off, it's only what type the
        smoothing will be if used which is controlled by this skill.

        By default a moving average is used.
    """
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
                    return self.success('Exponential smoothing enabled.')
                else:
                    return self.success('Exponential smoothing already enabled.')
            else:
                if self.exp_smoothing:
                    return self.success('Moving average enabled.')
                else:
                    return self.success('Moving average already enabled.')
        
        return self.fail('No reply from exp smooth.')
    
class WeightUpdate(SkillDescription):
    def createDescription(self):
        self.addParam('Last weight', 0.1, ParamTypes.Required)
    
class weight_update(PrimitiveThreadBase):
    """
    Summary:
        Updates the relative weight of the last measurement for the
        exponential smoothing.

    Required Input:
        Last weight: A positive float which is the weight the last measurement
                     will have.

    Behaviour:
        The relative weight the last measurement should have is published
        to the wrench relay.

        The first measurement (most recent) has a weight of 1 while the
        last measurement (oldest) has the specified weight. The rest of
        the measurements' weights are interpolated with an exponential
        curve and then normalized before computing the weighted average.
    """
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
                return self.success('Last weight value set.')
            else:
                return self.fail('Invalid weight value.', -1)
        
        return self.fail('No reply from weight update.', -1)

class SizeUpdate(SkillDescription):
    def createDescription(self):
        self.addParam('New size', 20, ParamTypes.Required)

class size_update(PrimitiveThreadBase):
    """
    Summary:
        Updates the amount of samples used in smoothing the signal.

    Required Input:
        New size: A positive integer

    Behaviour:
        Updates the amount of samples used in smoothing the
        force-torque signal in the wrench relay.
    """
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
                return self.success('Size of smoothing window updated.')
            else:
                return self.fail('Size of smoothing window needs to be positive.', -1)
        
        return self.fail('No reply from size update.', -1)

class ScaleUpdate(SkillDescription):
    def createDescription(self):
        self.addParam('Scale', 1.0, ParamTypes.Required)

class scale_update(PrimitiveThreadBase):
    """
    Summary:
        Updates the scaling of the force-torque signal

    Required Input:
        Scale: A positive float which the force-torque signal is scaled by.

    Behaviour:
        Both the force and the torque is scaled by the specified scale
        after the force-torque offset has been applied.

        To make the compliant controller more reactive to forces the
        scale can be increased and to make it less reactive to forces
        the scale can be decreased.
    """
    def createDescription(self):
        self.setDescription(ScaleUpdate(), self.__class__.__name__)

    def onInit(self):
        self.hz = 10
        self.time_limit = 1.5
        self.rate = rospy.Rate(self.hz)
        self.pub = rospy.Publisher('/cartesian_compliance_controller/scale', Float64, queue_size=1)
        self.sub = rospy.Subscriber('/cartesian_compliance_controller/scale_reply', Empty, callback=self.reply_callback)
    
    def preStart(self):
        self.reply = False
        self.running = True
        return True

    def reply_callback(self, _):
        if self.running:
            self.reply = True

    def run(self):
        scale = self.params['Scale'].value
        self.pub.publish(Float64(scale))

        count = 0
        while not self.reply and count < self.time_limit * self.hz:
            self.rate.sleep()
            count += 1

        self.running = False

        if self.reply:
            return self.success('Scale updated to %f.' % scale)
         
        return self.fail('No reply from scale update.', -1)
