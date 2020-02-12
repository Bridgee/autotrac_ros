#!/usr/bin/python

from datetime import datetime
import time
import rospy
import math
import struct
import numpy as np
import cv2
from sensor_msgs.msg import CompressedImage

from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()


def img_callback(msg):
	try:
		img = bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
	except CvBridgeError as e:
		print(e)
	else:
		cv2.imshow('raspicam_node visualization', img)
		# cv2.waitKey(20)
		# time.sleep(1)
		filename = 'img_dataset5/' + str(datetime.now()) + '.png'
		# filename = '2/view_img.png'
		cv2.imwrite(filename, img)


def main():
	img_topic = '/raspicam_node/image/compressed'

	rospy.init_node('save_imgFile')
	rospy.Subscriber(img_topic, CompressedImage, img_callback)
	rospy.spin()

if __name__ == '__main__':
	main()
