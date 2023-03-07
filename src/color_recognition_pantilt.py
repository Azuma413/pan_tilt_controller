#!/usr/bin/env python
# -*- coding: utf-8 -*-

from json.encoder import INFINITY
import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image, CompressedImage
import cv2
import numpy as np
import math
from cv_bridge import CvBridge
import sys

MIN_IMAGE_SIZE = 40
CAMERA_CENTER_X = 160
CAMERA_CENTER_Y = 120

class detect_object:

    def __init__(self, color_type, image, debug):
        self.color_type = color_type
        self.image_use = image
        self.debug = debug
        
    def object_detect(self, image):
        # Reference of this algorithm: https://qiita.com/seigot/items/008c95306dbd99a07309
        # Convert image into HSV format
        # About HSV: https://ja.wikipedia.org/wiki/HSV%E8%89%B2%E7%A9%BA%E9%96%93

        # Convert image from ROS-type to Opencv-type
        # np_arr = np.fromstring(image.data, np.uint8)
        # image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        image = bridge.imgmsg_to_cv2(image, 'bgr8')
        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
        # Hue
        h = hsv[:, :, 0]
        # Saturation (Chroma)
        s = hsv[:, :, 1]
        # Value (Brightness)
        v = hsv[:, :, 2]
        
        # red detection
        if self.color_type == 'RED':
            mask = np.zeros(h.shape, dtype=np.uint8)
            mask[((h < 10) | (h > 230)) & (s > 50)] = 255
            
        # blue detection
        if self.color_type == 'BLUE':
            lower_blue = np.array([130, 50, 25])
            upper_blue = np.array([200, 255, 255])
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            
        # green detection
        if self.color_type == 'GREEN':
            lower_green = np.array([70, 50, 50])
            upper_green = np.array([120, 255, 255])
            mask = cv2.inRange(hsv, lower_green, upper_green)
            
        # The definition of neighborhood
        neighborhood = np.array([[1, 1, 1, 1, 1],
                                 [1, 1, 1, 1, 1],
                                 [1, 1, 1, 1, 1],
                                 [1, 1, 1, 1, 1],
                                 [1, 1, 1, 1, 1]],
                                np.uint8)
        
        # Morphology -> Contraction and Expansion
        mask_morphology = mask
        
        for iter in range(5):
            mask_morphology = cv2.morphologyEx(mask_morphology, cv2.MORPH_OPEN, neighborhood)
            
#        if str(self.debug) == str('TRUE'):
#            cv2.imshow("Morphology Mask", mask_morphology)
        
        contours, _ = cv2.findContours(mask_morphology, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        rects = []
        
        for contour in contours:
            approx = cv2.convexHull(contour)
            rect = cv2.boundingRect(approx)
            rects.append(np.array(rect))
        
        if str(self.image_use) ==  str('TRUE'):
            self.show_detected_object(image, rects)
        
            
    def show_detected_object(self, image, rects):
        if len(rects) > 0:
            rect = max(rects, key=(lambda x: x[2] * x[3]))
            print(rect)
            marker_position = Point()
            for rect in rects:
                if self.color_type == 'RED':
                    cv2.rectangle(image, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 0, 255), thickness=2)
                elif self.color_type == 'GREEN':
                    cv2.rectangle(image, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 255, 0), thickness=2)
                elif self.color_type == 'BLUE':
                    cv2.rectangle(image, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (255, 0, 0), thickness=2)        
                
                marker_position.x = rect[0] + rect[2]/2 - CAMERA_CENTER_X
                marker_position.y = -(rect[1] + rect[3]/2 - CAMERA_CENTER_Y)
                marker_position.z = 0
                pub2.publish(marker_position)

#        cv2.imshow("Detected Objects", image)
        object_image = bridge.cv2_to_imgmsg(image, "bgr8")
        pub.publish(object_image)

if __name__=='__main__':

    rospy.init_node('Object_Detection', anonymous=True)
    rospy.loginfo('Object_Detection Node Initialized.')
    
    db = detect_object(color_type = 'BLUE', image = 'TRUE', debug = 'FALSE')
    
    bridge = CvBridge()

    pub = rospy.Publisher("Detected_Object_Image", Image, queue_size=10)
    pub2 = rospy.Publisher("Detected_Object_Position", Point, queue_size=10)
    
    rospy.Subscriber('image_raw', Image, db.object_detect)
    
    rospy.spin()
