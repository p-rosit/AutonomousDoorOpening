from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.params import ParamTypes
from skiros2_common.core.primitive_thread import PrimitiveThreadBase

import rospy
from std_msgs.msg import Empty, Bool, Int32, Float64

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
                return self.success('Changed state.')
            else:
                return self.success('Already in desired state')
        
        return self.fail('No reply from wrench.', -1)

class AdjustForce(SkillDescription):
    def createDescription(self):
        self.addParam('Adjust', False, ParamTypes.Required)

class adjust_force(PrimitiveThreadBase):
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
