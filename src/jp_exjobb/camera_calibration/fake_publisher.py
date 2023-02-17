#! /usr/bin/env python

import os
import cv2 as cv

import rospy
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image

from time import sleep

if __name__ == '__main__':
    # topic = '/real_sense/rgb/image_raw'
    topic = '/img'

    bridge = CvBridge()

    # load images from check and convert with cv_bridge
    images = []
    print(os.getcwd())
    path = os.path.join('src', 'jp_exjobb', 'camera_calibration', 'check')
    for name in os.listdir(path):
        if '.png' in name:
            img = cv.imread(os.path.join(path, name))
            img_msg = bridge.cv2_to_imgmsg(img, encoding='passthrough')
            images.append(img_msg)

    rospy.init_node('fake_image_publisher')
    ind = 0
    pub = rospy.Publisher(topic, Image, queue_size=1)

    def publish_callback(_):
        global ind
        ind = (ind + 1) % len(images)

    rate = rospy.Rate(0.1)
    sub = rospy.Subscriber('/camera_calibration/take_picture', String, callback=publish_callback)

    while not rospy.is_shutdown():
        print("Publishing image:", ind)
        pub.publish(images[ind])
        sleep(1)
