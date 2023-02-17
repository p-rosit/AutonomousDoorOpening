#! /usr/bin/env python

import rospy
from cv_bridge import CvBridge
from std_msgs.msg import  Empty, String, Float64MultiArray, Int32MultiArray
from sensor_msgs.msg import Image

from calibration import calibrate_camera

ok_status = "Ok"
nostart_status = "NoStart"
warning_status = "Warning"

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

class CameraCalibrationServer:
    def __init__(self):
        self.name = 'CameraCalibrationServer: '
        self.camera_topic = None
        self.camera_subscriber = None
        self.height, self.width = None, None

        # Publisher which responds to requests
        self.response_pub = rospy.Publisher('/camera_calibration/response', String, queue_size=1)
        self.calibration_pub = rospy.Publisher('/camera_calibration/calibration_parameters', Float64MultiArray, queue_size=1)

        # Topics and subscribers that control the state of the server
        self.start_topic    = '/camera_calibration/start'
        self.picture_topic  = '/camera_calibration/take_picture'
        self.delete_topic   = '/camera_calibration/delete'
        self.compute_topic  = '/camera_calibration/compute_calibration'
        self.start_sub      = rospy.Subscriber(self.start_topic,    String, callback=self.start_callback)
        self.picture_sub    = rospy.Subscriber(self.picture_topic,  String, callback=self.picture_callback)
        self.delete_sub     = rospy.Subscriber(self.delete_topic,   String, callback=self.delete_callback)
        self.compute_sub    = rospy.Subscriber(self.compute_topic,  String, callback=self.compute_callback)
        self.dimension_sub  = rospy.Subscriber('/camera_calibration/dimensions', Int32MultiArray, callback=self.dimension_callback)
        rospy.loginfo(self.name + 'Server started, waiting for signals.')

        self.started = False
        self.taking_pictures = False

    def respond(self, responding_to_topic, status):
        msg = String()
        msg.data = responding_to_topic + ":" + status

        self.response_pub.publish(msg)

    def dimension_callback(self, msg):
        self.height, self.width = msg.data

    def start_callback(self, msg):
        if self.started:
            if msg.data == self.camera_topic:
                rospy.loginfo(self.name + 'Start signal received but no finish signal was received. Clearing image list.')
                status = nostart_status
            else:
                rospy.logwarn(self.name + 'Start signal received but camera did not match existing camera. '
                                'Please complete current calibration before calibrating new camera. Ignoring signal.')
                self.respond(self.start_topic, warning_status)
                return
        else:
            rospy.loginfo(self.name + 'Start signal received.')
            status = ok_status

        self.camera_topic = msg.data
        self.camera_subscriber = RGBListener(topic=self.camera_topic)

        self.respond(self.start_topic, status)

        self.images = []
        self.started = True
        self.taking_pictures = True

    def picture_callback(self, _):
            
        if not self.taking_pictures:
            rospy.loginfo(self.name + 'Picture signal received but no start signal was received, ignoring signal.')
            self.respond(self.picture_topic, nostart_status)
            return
        
        rospy.loginfo(self.name + 'Picture signal received.')

        self.respond(self.picture_topic, ok_status)
        self.images.append(self.camera_subscriber.get())
    
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
        self.respond(self.delete_topic, ok_status)
        self.images.pop()

    def compute_callback(self, _):
        status = None
        if not self.started:
            rospy.loginfo(self.name + 'Compute calibration parameters signal received but no start signal was received, ignoring signal.')
            status = nostart_status
        elif not self.images:
            rospy.loginfo(self.name + 'Compute calibration parameters signal received but no pictures have been taken, ignoring signal.')
            status = warning_status
        elif not self.height or not self.width:
            rospy.loginfo(self.name + 'Compute calibration parameters signal received but heigth and width of checkerboard has not been set, ignoring signal.')
            status = warning_status

        if status is not None:
            self.respond(self.compute_topic, status)
            return

        self.camera_subscriber.unregister()
        self.camera_subscriber = None

        rospy.loginfo(self.name + 'Compute calibration parameters signal received, computing calibration parameters and publishing.')

        # Compute calibration parameters with opencv
        ret, calib, dist = calibrate_camera(self.images, 0.01, (self.height, self.width))

        calibration_parameters = [*calib, *dist]
        calibration_msg = Float64MultiArray()
        calibration_msg.data = calibration_parameters
        self.calibration_pub.publish(calibration_msg)

        self.respond(self.compute_topic, ok_status)
        self.started = False
        self.taking_pictures = False
        self.height, self.width = False, False


if __name__ == '__main__':
    rospy.init_node('camera_calibration_server')
    asss = CameraCalibrationServer()
    rospy.spin()
