from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive import PrimitiveBase
from skiros2_common.core.world_element import Element
from skiros2_common.core.params import ParamTypes
from skiros2_skill.core.skill import SkillDescription
from skiros2_common.core.primitive import PrimitiveBase

import rospy
from cv_bridge import CvBridge
from std_msgs.msg import Empty, String, Float64, Int32MultiArray
from sensor_msgs.msg import Image

from .calibration import calibrate_camera

import threading

ok_status = 'Ok'
noend_status = 'NoEnd'
nostart_status = 'NoStart'
warning_status = 'Warning'
clearing_status = 'Clearing'
nodim_status = 'NoDim'
nosize_status = 'NoSize'

class RGBListener:
    def __init__(self, topic='/realsense/rgb/image_raw'):
        self.time_limit = 2.0
        self.hz = 10
        self.bridge = CvBridge()
        self.rate = rospy.Rate(self.hz)
        self.image = 0.0

        self.sub = rospy.Subscriber(topic, Image, callback=self.image_callback)

    def unregister(self):
        self.sub.unregister()

    def image_callback(self, data):
        if self.image is not None: return

        self.image = self.bridge.imgmsg_to_cv2(data)

    def get(self):
        count = 0
        self.image = None

        while self.image is None and count < self.time_limit * self.hz:
            self.rate.sleep()
            count += 1
        
        if self.image is None:
            raise RuntimeError('No image recieved within %.2f seconds.' % self.time_limit)
        
        return self.image

class StartCalibration(SkillDescription):
    def createDescription(self):
        self.addParam('Camera', Element('skiros:DepthCamera'), ParamTypes.Required)
        self.addParam('Height', 5, ParamTypes.Required)
        self.addParam('Width', 8, ParamTypes.Required)
        self.addParam('Square size (m)', 0.02, ParamTypes.Required)

class CameraCalibration(SkillDescription):
    def createDescription(self):
        pass

class camera_calibration_skill(PrimitiveBase):
    def createDescription(self):
        self.setDescription(CameraCalibration(), self.__class__.__name__)

    def onInit(self):
        self.time_limit = 5.0
        self.running = False
        self.response = False
        self.super_topic = '/camera_calibration'
        self.sub = rospy.Subscriber(self.super_topic + '/response', String, callback=self.response_callback)

        self.ok_msg = 'Unreachable error ¯\_(ツ)_/¯'
        self.no_start_msg = 'Unreachable error ¯\_(ツ)_/¯'
        self.no_end_msg = 'Unreachable error ¯\_(ツ)_/¯'
        self.warning_msg = 'Unreachable error ¯\_(ツ)_/¯'
        self.no_dim_msg = 'Unreachable error ¯\_(ツ)_/¯'
        self.no_size_msg = 'Unreachable error ¯\_(ツ)_/¯'

        return True

    def response_callback(self, msg):
        if self.running:
            topic, status = msg.data.split(':')
            if topic == self.sub_topic:
                self.response = True
                self.status = status
            else:
                rospy.logwarn('Incorrect response received, are several camera calibration skills running?')

    def onStart(self):
        self.running = False
        self.response = False
        return True

    def execute(self):
        if not self.running:
            self.running = True
            self.start_time = rospy.Time.now()
            self.pub.publish(Empty())
            return self.step('Message sent.')
        else:
            if self.response:
                if self.status == ok_status:
                    return self.success(self.ok_msg)
                elif self.status == nostart_status:
                    return self.fail(self.no_start_msg, -1)
                elif self.status == warning_status:
                    return self.fail(self.warning_msg, -1)
                elif self.status == nodim_status:
                    return self.fail(self.no_dim_msg, -1)
                elif self.status == nosize_status:
                    return self.fail(self.no_size_msg, -1)
                else:
                    return self.fail('Unknown status "%s", unreachable error ¯\_(ツ)_/¯' % self.status, -1)
            elif rospy.Duration(self.time_limit) < rospy.Time.now() - self.start_time:
                return self.fail('Calibration server did not respond within %.1f seconds.' % self.time_limit, -1)
            else:
                return self.step('Waiting for reply.')
    
    def onEnd(self):
        self.running = False
        return True

class start_calibration(camera_calibration_skill):
    def createDescription(self):
        self.setDescription(StartCalibration(), self.__class__.__name__)

    def onInit(self):
        self.sub_topic = '/start'
        super().onInit()
        self.pub = rospy.Publisher(self.super_topic + self.sub_topic, String, queue_size=1)
        self.camera_pub = rospy.Publisher(self.super_topic + '/camera_name', String, queue_size=1)
        self.dimension_pub = rospy.Publisher(self.super_topic + '/dimensions', Int32MultiArray, queue_size=1)
        self.size_pub = rospy.Publisher(self.super_topic + '/square_size', Float64, queue_size=1)
        self.msg = String()

        self.ok_msg = 'Camera calibration started.'
        self.no_start_msg = 'Unreachable error ¯\_(ツ)_/¯'
        self.no_end_msg = 'Image list cleared.'
        self.warning_msg = 'Calibration already started, cannot change camera.'
        self.no_dim_msg = 'Unreachable error ¯\_(ツ)_/¯'
        self.no_size_msg = 'Unreachable error ¯\_(ツ)_/¯'

        return True
    
    def execute(self):
        if not self.running:
            self.running = True
            self.start_time = rospy.Time.now()

            camera = self.params['Camera'].value
            self.msg.data = camera.getProperty('skiros:DriverAddress').value + "/rgb/image_raw"
            # self.msg.data = '/img'
            self.pub.publish(self.msg)

            self.msg.data = camera.id
            self.camera_pub.publish(self.msg)

            msg = Int32MultiArray()
            msg.data = [self.params['Height'].value, self.params['Width'].value]
            self.dimension_pub.publish(msg)

            msg = Float64()
            msg.data = self.params['Square size (m)'].value
            self.size_pub.publish(msg)

            return self.step('Message sent.')
        else:
            if self.response:
                if self.status == ok_status:
                    return self.success(self.ok_msg)
                elif self.status == nostart_status:
                    return self.fail(self.no_start_msg, -1)
                elif self.status == noend_status:
                    return self.success(self.no_end_msg)
                elif self.status == warning_status:
                    return self.fail(self.warning_msg, -1)
                elif self.status == nodim_status:
                    return self.fail(self.no_dim_msg, -1)
                elif self.status == nosize_status:
                    return self.fail(self.no_size_msg, -1)
                else:
                    return self.fail('Unknown status "%s", unreachable error ¯\_(ツ)_/¯' % self.status, -1)
            elif rospy.Duration(self.time_limit) < rospy.Time.now() - self.start_time:
                return self.fail('Calibration server did not respond within %.1f seconds.' % self.time_limit, -1)
            else:
                return self.step('Waiting for reply.')

class take_picture(camera_calibration_skill):
    def onInit(self):
        self.sub_topic = '/take_picture'
        super().onInit()
        self.pub = rospy.Publisher(self.super_topic + self.sub_topic, Empty, queue_size=1)

        self.ok_msg = 'Picture taken.'
        self.no_start_msg = 'No camera currently being calibrated.'

        return True

class delete_previous(camera_calibration_skill):
    def onInit(self):
        self.sub_topic = '/delete'
        super().onInit()
        self.pub = rospy.Publisher(self.super_topic + self.sub_topic, Empty, queue_size=1)

        self.ok_msg = 'Most recent image deleted.'
        self.no_start_msg = 'No camera currently being calibrated.'
        self.warning_msg = 'Image list already empty.'

        return True

class abort_calibration(camera_calibration_skill):
    def onInit(self):
        self.sub_topic = '/abort_calibration'
        super().onInit()
        self.pub = rospy.Publisher(self.super_topic + self.sub_topic, Empty, queue_size=1)

        self.ok_msg = 'Camera calibration aborted.'
        self.no_start_msg = 'No camera currently being calibrated.'

        return True

class calibrate(camera_calibration_skill):
    def onInit(self):
        self.sub_topic = '/compute_calibration'
        super().onInit()

        self.ok_msg = 'Camera parameters computed.'
        self.no_ok_msg = 'Camera could not be calibrated, opencv function failed.'
        self.no_start_msg = 'No camera currently being calibrated.'
        self.no_end_msg = 'Image list cleared.'
        self.warning_msg = 'No images have been taken, cannot calibrate.'
        self.no_dim_msg = 'Dimension of calibration pattern not set.'
        self.no_size_msg = 'Size of squares in calibration pattern not set.'

        self.name = 'CameraCalibrationServer: '
        self.camera_topic = None
        self.camera_name = None
        self.camera_subscriber = None
        self.height, self.width = None, None
        self.square_size = None

        # Publisher which responds to requests
        self.response_pub = rospy.Publisher(self.super_topic + '/response', String, queue_size=1)
        self.msg = String()

        # Topics and subscribers that control the state of the server
        self.start_topic    = '/start'
        self.picture_topic  = '/take_picture'
        self.delete_topic   = '/delete'
        self.abort_topic    = '/abort_calibration'
        self.start_sub      = rospy.Subscriber(self.super_topic + self.start_topic,   String, callback=self.start_callback)
        self.picture_sub    = rospy.Subscriber(self.super_topic + self.picture_topic, Empty,  callback=self.picture_callback)
        self.delete_sub     = rospy.Subscriber(self.super_topic + self.delete_topic,  Empty,  callback=self.delete_callback)
        self.abort_sub      = rospy.Subscriber(self.super_topic + self.abort_topic,   Empty,  callback=self.abort_callback)
        
        # Information topics
        self.name_sub = rospy.Subscriber(self.super_topic + '/camera_name', String, callback=self.name_callback)
        self.dimension_sub = rospy.Subscriber(self.super_topic + '/dimensions', Int32MultiArray, callback=self.dimension_callback)
        self.size_sub = rospy.Subscriber(self.super_topic + '/square_size', Float64, callback=self.size_callback)

        self.started = False
        self.taking_pictures = False
        return True

    def respond(self, responding_to_topic, status):
        self.msg.data = responding_to_topic + ":" + status
        self.response_pub.publish(self.msg)

    def name_callback(self, msg):
        if self.camera_name is None:
            self.camera_name = msg.data
        else:
            if self.camera_name != msg.data:
                rospy.logwarn(self.name + 'Camera name received but camera did not match existing camera. '
                                'Please complete or abort current calibration before calibrating new camera. Ignoring signal.')

    def dimension_callback(self, msg):
        self.height, self.width = msg.data

    def size_callback(self, msg):
        self.square_size = msg.data

    def start_callback(self, msg):
        if self.started:
            if msg.data == self.camera_topic:
                rospy.loginfo(self.name + 'Start signal received but no finish signal was received. Clearing image list.')
                status = noend_status
            else:
                rospy.logwarn(self.name + 'Start signal received but camera did not match existing camera. '
                                'Please complete current calibration before calibrating new camera. Ignoring signal.')
                self.respond(self.start_topic, warning_status)
                return
        else:
            rospy.loginfo(self.name + 'Start signal received.')
            status = ok_status

        self.images = []
        self.started = True
        self.taking_pictures = True

        self.camera_topic = msg.data
        self.camera_subscriber = RGBListener(topic=self.camera_topic)

        self.respond(self.start_topic, status)

    def picture_callback(self, _):
        if not self.taking_pictures:
            rospy.loginfo(self.name + 'Picture signal received but no start signal was received, ignoring signal.')
            self.respond(self.picture_topic, nostart_status)
            return
        
        rospy.loginfo(self.name + 'Picture signal received.')

        self.images.append(self.camera_subscriber.get())
        self.respond(self.picture_topic, ok_status)

    def delete_callback(self, _):
        if not self.taking_pictures:
            rospy.loginfo(self.name + 'Delete signal received but no start signal was received, ignoring signal.')
            self.respond(self.delete_topic, nostart_status)
            return
        if not self.images:
            rospy.loginfo(self.name + 'Delete signal received but image list is empty, ignoring signal.')
            self.respond(self.delete_topic, warning_status)
            return
        
        rospy.loginfo(self.name + 'Delete signal received, deleting most recent image.')
        self.images.pop()
        self.respond(self.delete_topic, ok_status)

    def clear_camera_info(self):
        self.camera_subscriber.unregister()
        self.camera_subscriber = None

        self.camera_name = None
        self.started = False
        self.taking_pictures = False
        self.height, self.width = False, False
        self.square_size = None

    def abort_callback(self, _):
        if not self.taking_pictures:
            rospy.loginfo(self.name + 'Abort signal received but no start signal was received, ignoring signal.')
            self.respond(self.abort_topic, nostart_status)
            return

        rospy.loginfo(self.name + 'Abort signal received, clearing camera info without writing to world model.')
        self.clear_camera_info()
        self.respond(self.abort_topic, ok_status)

    def extract_camera_parameters(self):
        camera = self.wmi.get_element(self.camera_name)

        camera_relations = camera.getRelations(pred=["skiros:hasA"])
        for relation in camera_relations:
            object = self.wmi.get_element(relation['dst'])
            if object.type == 'scalable:CalibrationParameters':
                calibration_params = object
                break
        
        return calibration_params

    def compute_parameters(self):
        self.status = None
        if not self.started:
            rospy.loginfo(self.name + 'Compute calibration parameters signal received but no start signal was received, ignoring signal.')
            self.status = nostart_status
        elif not self.images:
            rospy.loginfo(self.name + 'Compute calibration parameters signal received but no pictures have been taken, ignoring signal.')
            self.status = warning_status
        elif not self.height or not self.width:
            rospy.logwarn(self.name + 'Compute calibration parameters signal received but heigth and width of checkerboard has not been set. Unreachable error ignoring signal.')
            self.status = warning_status
        elif not self.square_size:
            rospy.logwarn(self.name + 'Compute calibration parameters signal received but square size has not been set. Unreachable error, ignoring signal.')
            self.status = warning_status

        if self.status is not None:
            self.calibration_completed = True
            self.succeeded = False
            return

        rospy.loginfo(self.name + 'Compute calibration parameters signal received, computing calibration parameters and publishing.')

        # Compute calibration parameters with opencv
        # correct square size
        ret, calib, dist = calibrate_camera(self.images, self.square_size, (self.height, self.width))

        # calib = (1.0, 2.0, 3.0, 4.0)
        # dist = (1.0, 2.0, 3.0, 4.0, 5.0)

        fx, fy, cx, cy = calib
        k1, k2, p1, p2, k3 = dist

        calibration_params = self.extract_camera_parameters()
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

        self.calibration_completed = True
        self.succeeded = ret
        self.status = ok_status

        self.clear_camera_info()

    def onStart(self):
        super().onStart()
        self.thread = threading.Thread(target=self.compute_parameters)
        self.thread.start()
        return True

    def execute(self):
        if not self.running:
            self.running = True
            self.calibration_completed = False
            return self.step('Computing parameters.')
        
        if self.calibration_completed:
            self.thread.join()
            if self.status == ok_status:
                if self.succeeded:
                    return self.success(self.ok_msg)
                else:
                    return self.fail(self.no_ok_msg, -1)
            elif self.status == nostart_status:
                return self.fail(self.no_start_msg, -1)
            elif self.status == noend_status:
                return self.success(self.no_end_msg)
            elif self.status == warning_status:
                return self.fail(self.warning_msg, -1)
            elif self.status == nodim_status:
                return self.fail(self.no_dim_msg, -1)
            elif self.status == nosize_status:
                return self.fail(self.no_size_msg, -1)
            else:
                return self.fail('Unknown status "%s", unreachable error ¯\_(ツ)_/¯' % self.status, -1)
        
        return self.step('Computing parameters.')
