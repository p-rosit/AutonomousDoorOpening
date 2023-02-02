from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes
import rospy
from std_msgs.msg import Empty, String

from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_std_skills.action_client_primitive import PrimitiveActionClient
import rospy
import actionlib
from jp_exjobb.msg import EmptyAction, EmptyGoal, EmptyResult, CameraCalibrationMsg
from std_msgs.msg import Int32, String, Float64MultiArray

ok_status = "Ok"
nostart_status = "NoStart"
warning_status = "Warning"
clearing_status = "Clearing"

class CameraCalibration(SkillDescription):
    def createDescription(self):
        self.addParam('Camera', Element('skiros:DepthCamera'), ParamTypes.Required)

class camera_calibration_topic(PrimitiveBase):

    def createDescription(self):
        self.setDescription(CameraCalibration(), self.__class__.__name__)
    
    # self.topic and self.message need to be set in onInit by class extending this
    def onInit(self):
        self.running = False
        self.time_limit = 2
        self.hz = 10
        self.rate = rospy.Rate(self.hz)
        self.pub = rospy.Publisher(self.topic, String, queue_size=1)
        self.sub = rospy.Subscriber('/camera_calibration/response', String, callback=self.reponse_callback)
        return True
    
    def reponse_callback(self, msg):
        if self.running:
            topic, status = msg.data.split(':')
            if topic == self.topic:
                self.response = True
                self.status = status
            else:
                rospy.logwarn('Incorrect response received, are several camera calibration skills running?')

    def onPreempt(self):
        return True
    
    def onStart(self):
        self.running = True
        self.response = False
        return True

    def execute(self):
        camera = self.params['Camera'].value
        msg = String()
        msg.data = camera.getProperty('skiros:DriverAddress').value + "/rgb/image_raw"
        self.pub.publish(msg)

        count = 0
        while not self.response and count < self.time_limit * self.hz:
            self.rate.sleep()
            count += 1

        if self.response:
            return self.success(self.message)
        else:
            return self.fail('Camera calibration action server did not respond.', -1)
    
    def onEnd(self):
        self.running = False
        return True

class start_camera_calibration(camera_calibration_topic):
    def onInit(self):
        self.topic = '/camera_calibration/start'
        self.message = 'Camera calibration action server started.'
        self.camera_pub = rospy.Publisher('/camera_calibration/camera_name', String, queue_size=1)
        return super().onInit()
    
    def execute(self):
        camera = self.params['Camera'].value
        msg = String()
        msg.data = camera.id
        self.camera_pub.publish(msg)

        return super().execute()

    def reponse_callback(self, msg):
        status = super().reponse_callback(msg)

class take_picture(camera_calibration_topic):
    def onInit(self):
        self.topic = '/camera_calibration/take_picture'
        self.message = 'Picture taken.'
        
        ok_status = "Ok"
        nostart_status = "NoStart"
        warning_status = "Warning"
        clearing_status = "Clearing"
        
        
        self.warn_message = 'kiss'
        self.ok_message = 'bajs'
        return super().onInit()

class compute_intrinsic_camera_parameters(camera_calibration_topic):
    def onInit(self):
        self.started = True
        self.camera_name = None
        self.parameters = None
        
        self.topic = '/camera_calibration/compute_calibration'
        self.message = 'Intrinsic camera parameters computed.'

        self.start_sub = rospy.Subscriber('/camera_calibration/start', String, callback=self.start_callback)
        self.camera_sub = rospy.Subscriber('/camera_calibration/camera_name', String, callback=self.name_callback)
        self.calibration_sub = rospy.Subscriber('/camera_calibration/calibration_parameters', Float64MultiArray, callback=self.calibration_callback)
        return super().onInit()

    def start_callback(self, _):
        self.started = True

    def name_callback(self, msg):
        if not self.started:
            if self.camera_name != msg.data:
                rospy.logwarn('CameraCalibrationServer: Start signal received but camera did not match existing camera. '
                                'Please complete current calibration before calibrating new camera. Ignoring signal.')
                return
        else:
            self.camera_name = msg.data

    def calibration_callback(self, msg):
        self.parameters = msg.data
    
    def execute(self):
        self.started = False
        super().execute()
        if self.response:
            camera = self.wmi.get_element(self.camera_name)

            camera_relations = camera.getRelations(pred=["skiros:hasA"])
            for relation in camera_relations:
                object = self.wmi.get_element(relation['dst'])
                if object.type == 'scalable:CalibrationParameters':
                    calibration_params = object
                    break

            fx, fy, cx, cy, k1, k2, p1, p2, k3 = self.parameters

            calibration_params.setProperty('scalable:FocalLengthX', fx)
            calibration_params.setProperty('scalable:FocalLengthY', fy)
            calibration_params.setProperty('scalable:PixelCenterX', cx)
            calibration_params.setProperty('scalable:PixelCenterY', cy)
            calibration_params.setProperty('scalable:Distortionk1', k1)
            calibration_params.setProperty('scalable:Distortionk2', k2)
            calibration_params.setProperty('scalable:Distortionp1', p1)
            calibration_params.setProperty('scalable:Distortionp2', p2)
            calibration_params.setProperty('scalable:Distortionk3', k3)

            self.wmi.update_element(calibration_params)

            return self.success('bajs2')
        else:
            return self.fail('Camera calibration action server did not respond.', -1)

class delete_previous_image(camera_calibration_topic):
    def onInit(self):
        self.topic = '/camera_calibration/delete'
        self.message = 'Deleted most recent image.'
        return super().onInit()
