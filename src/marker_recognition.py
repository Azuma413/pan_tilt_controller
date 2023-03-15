#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from json.encoder import INFINITY
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point
import cv2
from cv_bridge import CvBridge
import numpy as np
import math
import sys

MIN_IMAGE_SIZE = 40
#CAMERA_CENTER_X = 160
#CAMERA_CENTER_Y = 120
CAMERA_CENTER_X = 640
CAMERA_CENTER_Y = 360

class ColorRecognition:

    def __init__(self, color_type, use_image_pub):
        # ROSからOpenCVに画像形式を変換する用のブリッジinstanceを生成
        self.bridge = CvBridge()

        # 認識するマーカーの色
        self.color_type = color_type
        # 画像をpublishするかどうかの変数
        self.use_image_pub = use_image_pub
        # 検出したマーカー位置を格納する変数
        self.marker_pos = None
        # 検出したマーカーを含む画像を格納する変数
        self.marker_img = None
        
    def recognize_color(self, image):
        # Reference of this algorithm: https://qiita.com/seigot/items/008c95306dbd99a07309
        # Convert image into HSV format
        # About HSV: https://ja.wikipedia.org/wiki/HSV%E8%89%B2%E7%A9%BA%E9%96%93

        # ROSの画像形式からOpenCVの画像形式に変換する
        image = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        # 画像フォーマットをBGRからHSVに変換する
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV_FULL)
        # Hue
        h = hsv[:, :, 0]
        # Saturation (Chroma)
        s = hsv[:, :, 1]
        # Value (Brightness)
        v = hsv[:, :, 2]
        
        # 赤色のマーカー検出の場合の処理
        if self.color_type == 'RED':
            mask = np.zeros(h.shape, dtype=np.uint8)
            mask[((h < 10) | (h > 230)) & (s > 50)] = 255
            
        # 青色のマーカー検出の場合の処理
        if self.color_type == 'BLUE':
            lower_blue = np.array([130, 50, 25])
            upper_blue = np.array([200, 255, 255])
            mask = cv2.inRange(hsv, lower_blue, upper_blue)
            
        # 緑色のマーカー検出の場合の処理
        if self.color_type == 'GREEN':
            lower_green = np.array([70, 50, 50])
            upper_green = np.array([120, 255, 255])
            mask = cv2.inRange(hsv, lower_green, upper_green)
            
        # マスクの大きさを定義
        neighborhood = np.array([[1, 1, 1, 1, 1],
                                 [1, 1, 1, 1, 1],
                                 [1, 1, 1, 1, 1],
                                 [1, 1, 1, 1, 1],
                                 [1, 1, 1, 1, 1]],
                                np.uint8)
        
        # Morphology -> Contraction and Expansion
        mask_morphology = mask
        
        # オープニングモルフォロジー演算を繰り返して, 画像をフィルタリングする処理
        for iter in range(5):
            mask_morphology = cv2.morphologyEx(mask_morphology, cv2.MORPH_OPEN, neighborhood)
        # 輪郭を検出する処理
        contours, _ = cv2.findContours(mask_morphology, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        # 検出したマーカー座標格納用変数
        rects = []
        # 輪郭の検出
        for contour in contours:
            approx = cv2.convexHull(contour)
            rect = cv2.boundingRect(approx)
            rects.append(np.array(rect))
        
        # マーカーが少なくとも1つ検出されている場合の処理
        if len(rects) > 0:
            # マーカーのうち, 輪郭の面積最大のものを取り出す.
            rect = max(rects, key=(lambda x: x[2] * x[3]))
            # 赤色検出の場合
            if self.color_type == 'RED':
                cv2.rectangle(image, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 0, 255), thickness=2)
            # 緑色検出の場合
            elif self.color_type == 'GREEN':
                cv2.rectangle(image, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (0, 255, 0), thickness=2)
            # 青色検出の場合
            elif self.color_type == 'BLUE':
                cv2.rectangle(image, tuple(rect[0:2]), tuple(rect[0:2] + rect[2:4]), (255, 0, 0), thickness=2)        
            
            # ROSのデータ型を用意
            marker_pos = Point()
            # カメラの画素数を考慮して, マーカーの位置を計算
            marker_pos.x = ((rect[0] + rect[2]/2) - CAMERA_CENTER_X)
            marker_pos.y = - ((rect[1] + rect[3]/2) - CAMERA_CENTER_Y)
            marker_pos.z = 0
            self.marker_pos = marker_pos
            # OpenCVからROSの画像形式に変換
            self.marker_img = self.bridge.cv2_to_imgmsg(image, "bgr8")
            
    # 検出したマーカー位置を外部に渡す関数
    def detected_marker_pos(self):
        return self.marker_pos

    # 検出したマーカーを含む画像を外部に渡す関数
    def detected_marker_img(self):
        return self.marker_img

if __name__=='__main__':
    try:
        # ColorRecognition node の生成
        rospy.init_node('ColorRecognition', anonymous=True)
        rospy.loginfo('ColorRecognition Initialized.')

        # ColorRecognitionのインスタンスを生成
        cr = ColorRecognition(color_type = rospy.get_param("marker_color"), use_image_pub = True)
        
        # 検出したマーカー位置のpublisher
        pub_marker_pos = rospy.Publisher("detected_marker_pos", Point, queue_size=10)
        # 検出したマーカーを含んだ画像のpublisher
        pub_marker_img = rospy.Publisher("detected_marker_img", Image, queue_size=10)
        
        # tb3に取り付けられたカメラからの画像を取得するsubscriber
        rospy.Subscriber('camera/color/image_raw', Image, cr.recognize_color)
        
        # whileループの周期を決定
        r = rospy.Rate(10)

        while not rospy.is_shutdown():
            # 検出マーカーの位置をpublish
            if cr.detected_marker_pos() != None:
                pub_marker_pos.publish(cr.detected_marker_pos())
            # 検出マーカーと含む画像をpublish
            if cr.detected_marker_img() != None:
                pub_marker_img.publish(cr.detected_marker_img())

            r.sleep()

    except rospy.ROSInterruptException: pass
