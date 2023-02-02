#! /usr/bin/env python

import rospy
from std_msgs.msg import Empty, String


class CameraCalibrationServer:
    def __init__(self):
        self.name = 'CameraCalibrationServer: '

        # Publisher which responds to requests
        self.response_pub = rospy.Publisher('/camera_calibration/response', String, queue_size=1)

        # Topics that control the state of the server
        self.start_topic = '/camera_calibration/start'
        self.picture_topic = '/camera_calibration/take_picture'
        self.compute_topic = '/camera_calibration/compute_calibration'
        self.start_sub = rospy.Subscriber(self.start_topic, Empty, callback=self.start_callback)
        self.picture_sub = rospy.Subscriber(self.picture_topic, Empty, callback=self.picture_callback)
        self.compute_sub = rospy.Subscriber(self.compute_topic, Empty, callback=self.compute_callback)

        rospy.loginfo(self.name + 'Server started, waiting for signals.')

        self.started = False
        self.taking_pictures = False

    def respond(self, responding_to_topic):
        msg = String()
        msg.data = responding_to_topic

        self.response_pub.publish(msg)

    def start_callback(self, _):
        self.respond(self.start_topic)

        if self.started:
            rospy.loginfo(self.name + 'Start signal recieved but not finish signal was recieved. Clearing image list.')
        else:
            rospy.loginfo(self.name + 'Start signal recieved.')

        self.images = []
        self.started = True
        self.taking_pictures = True

    def picture_callback(self, _):
        self.respond(self.picture_topic)
            
        if not self.taking_pictures:
            rospy.loginfo(self.name + 'Picture signal recieved but no start signal was recieved, ignoring signal.')
            return
        
        rospy.loginfo(self.name + 'Picture signal recieved.')

        # add image to self.images
        self.images.append(1)
     
    def compute_callback(self, _):
        self.respond(self.compute_topic)
        
        if not self.started:
            rospy.loginfo(self.name + 'Compute calibration parameters signal recieved but no start signal was recieved, ignoring signal.')
            return
        elif not self.images:
            rospy.loginfo(self.name + 'Compute calibration parameters signal recieved but no pictures have been taken, ignoring signal.')
            return

        rospy.loginfo(self.name + 'Compute calibration parameters signal recieved, computing calibration parameters and publishing.')

        # Compute calibration parameters with opencv

        self.started = False
        self.taking_pictures = False


if __name__ == '__main__':
    rospy.init_node('camera_calibration_server')
    asss = CameraCalibrationServer()
    rospy.spin()
