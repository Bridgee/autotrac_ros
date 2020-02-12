#!/usr/bin/python

import sys
import rospy
from sensor_msgs.msg import CompressedImage
from raspicam_node.msg import LaneDetection
from cv_bridge import CvBridge, CvBridgeError

import cv2
from PIL import Image
import numpy as np
import glob


bridge = CvBridge()
ld_pub = rospy.Publisher('lane_detection', LaneDetection, queue_size = 1)


def img_processing(img):
	# Opens a image in RGB mode  
	cv_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
	pil_im = Image.fromarray(cv_img)
	width, height = pil_im.size 
 
	# Setting the points for cropped image  
	left = 22
	top = 130
	right = 372
	bottom = 308

	# Cropped image of above dimension  
	# (It will not change orginal image)  
	im1 = pil_im.crop((left, top, right, bottom))
	newsize = (105, 54) 
	im1 = im1.resize(newsize, Image.BICUBIC) 

	# Binarize
	grayImg = im1.convert('L')
	# im_np = np.asarray(grayImg)
	# cv2.imshow('grayImg', im_np)
	# cv2.waitKey(3)
	BW = np.asarray(grayImg)
    
	BW1 = (BW > 0.6*255).astype(int)
	RGB = np.asarray(im1)
	R = RGB[...,0]
	G = RGB[...,1]
	B = RGB[...,2]
	BW1[(G >= 100) & (R <= 20)] = 1;
	BW1[(R >= 150) & (G >= 150) & (B <= 100)] = 0;
	BW1[((R.astype('float64') - G.astype('float64')) >= 80) & ((R.astype('float64') - B.astype('float64')) >= 80)] = 0;
	BW1 = np.absolute(BW1-1);
	centerInd = np.zeros(BW1.shape[0])
	for i in range(BW1.shape[0]):
		centerInd[i] = np.sum(np.dot(np.arange(BW1.shape[1]), BW1[i,:]))/np.sum(BW1[i,:]);
	CenterPoint = round(np.median(centerInd[np.isnan(centerInd) == 0]));
	steering_error = np.arctan((np.median(centerInd[np.isnan(centerInd) == 0])+1-(105/2))/27)/np.pi*180;
	at_curve = False
	
	return steering_error, at_curve



def img_callback(data):
	try:
		cv_img = bridge.compressed_imgmsg_to_cv2(data, 'bgr8')
	except CvBridgeError as e:
		print(e)
	else:
		global verbose
		if verbose:
			cv2.imshow('raspicam_node visualization', cv_img)
			cv2.waitKey(3)

		steering_error, at_curve = img_processing(cv_img)

		# Publish the processed data
		msg = LaneDetection()
		msg.steering_error = steering_error
		msg.at_curve = at_curve

		# while not rospy.is_shutdown():
		ld_pub.publish(msg)
		rospy.loginfo(msg)


		


def laneDetection():
	img_topic = '/raspicam_node/image/compressed'

	rospy.init_node('laneDetection')
	rospy.Subscriber(img_topic, CompressedImage, img_callback)
	rospy.spin()



if __name__ == '__main__':
	global verbose
	if len(sys.argv) != 2:
		verbose = False
	else:
		verbose = sys.argv[1] == 'verbose'
	# print("verbose = ", verbose)

	laneDetection()