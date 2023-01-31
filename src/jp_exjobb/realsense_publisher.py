#! /usr/bin/env python

from time import time

import rospy
from sensor_msgs.msg import Image

import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
import pyrealsense2 as rs

def has_rgb(device):
	if 'RGB Camera' in [s.get_info(rs.camera_info.name) for s in device.sensors]:
		return True
	return False

if __name__== '__main__':

	rospy.init_node('picture')

	bridge = CvBridge()

	pub = rospy.Publisher('/img', Image, queue_size=1)

	pipeline = rs.pipeline()
	config = rs.config()

	wrapper = rs.pipeline_wrapper(pipeline)
	wrapper_profile = config.resolve(wrapper)
	device = wrapper_profile.get_device()

	product_line = str(device.get_info(rs.camera_info.product_line))

	if not has_rgb(device):
		raise RunTimeError('Failed to find RGB camera')

	if product_line == 'L500':
		# config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30) # USB 3.0
		config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30) # USB 2.1
	else:
		config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

	align = rs.align(rs.stream.color)
	profile = pipeline.start(config)

	try:
		start = time()
		while not rospy.is_shutdown():
			frame = pipeline.wait_for_frames()
			aligned = align.process(frame)

			rgb_frame = aligned.get_color_frame()
			rgb = np.asanyarray(rgb_frame.get_data())

			dt = time() - start

			if dt >= 0.5:
				# publish picture
				# lite rum
				img_msg = bridge.cv2_to_imgmsg(rgb, encoding='passthrough')
				pub.publish(img_msg)
				#ett spacelite mer rum

				#cv.imshow('test_window', rgb)
				#key = cv.waitKey(3) & 0xff
				#if key == ord('q'):
				#	break
				start = time()
	finally:
		pipeline.stop()

