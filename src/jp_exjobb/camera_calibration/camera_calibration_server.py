#! /usr/bin/env python

import rospy
from std_msgs.msg import  String, Float64MultiArray

ok_status = "Ok"
nostart_status = "NoStart"
warning_status = "Warning"

class CameraCalibrationServer:
    def __init__(self):
        self.name = 'CameraCalibrationServer: '
        self.camera_topic = None

        # Publisher which responds to requests
        self.response_pub = rospy.Publisher('/camera_calibration/response', String, queue_size=1)
        self.camera_pub = rospy.Publisher('/camera_calibration/camera_name', String, queue_size=1)
        self.calibration_pub = rospy.Publisher('/camera_calibration/calibration_parameters', Float64MultiArray, queue_size=1)

        # Topics and subscribers that control the state of the server
        self.start_topic = '/camera_calibration/start'
        self.picture_topic = '/camera_calibration/take_picture'
        self.delete_topic = '/camera_calibration/delete'
        self.compute_topic = '/camera_calibration/compute_calibration'
        self.start_sub = rospy.Subscriber(self.start_topic, String, callback=self.start_callback)
        self.picture_sub = rospy.Subscriber(self.picture_topic, String, callback=self.picture_callback)
        self.delete_sub = rospy.Subscriber(self.delete_topic, String, callback=self.delete_callback)
        self.compute_sub = rospy.Subscriber(self.compute_topic, String, callback=self.compute_callback)

        rospy.loginfo(self.name + 'Server started, waiting for signals.')

        self.started = False
        self.taking_pictures = False

    def respond(self, responding_to_topic, status):
        msg = String()
        msg.data = responding_to_topic + ":" + status

        self.response_pub.publish(msg)

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
        # add image to self.images
        self.images.append(1)
    
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
        if not self.started:
            rospy.loginfo(self.name + 'Compute calibration parameters signal received but no start signal was received, ignoring signal.')
            self.respond(self.compute_topic, nostart_status)
            return
        elif not self.images:
            rospy.loginfo(self.name + 'Compute calibration parameters signal received but no pictures have been taken, ignoring signal.')
            self.respond(self.compute_topic, warning_status)
            return

        rospy.loginfo(self.name + 'Compute calibration parameters signal received, computing calibration parameters and publishing.')
        
        # Compute calibration parameters with opencv
        
        fx, fy, cx, cy, k1, k2, p1, p2, k3 = 1, 2, 3, 4, 5, 6, 7, 8, 9

        calibration_parameters = [fx, fy, cx, cy, k1, k2, p1, p2, k3]
        calibration_msg = Float64MultiArray()
        calibration_msg.data = calibration_parameters
        self.calibration_pub.publish(calibration_msg)

        self.respond(self.compute_topic, ok_status)
        self.started = False
        self.taking_pictures = False


if __name__ == '__main__':
    rospy.init_node('camera_calibration_server')
    asss = CameraCalibrationServer()
    rospy.spin()
