#! /usr/bin/python2.7
from __future__ import print_function

import rospy, roslib
from lane_detect.msg import SignInfo
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
from sign_detect import *
from sign_classification import *

def listener():
    bridge = CvBridge()
    
    rospy.init_node('signdetect', anonymous=True)

    pub = rospy.Publisher('/sign_detect', SignInfo, queue_size=1)

    sign_info = SignInfo()
    
    while not rospy.is_shutdown():
        image = rospy.wait_for_message("Team1_image/compressed", CompressedImage)

        # decompress image
        np_arr = np.fromstring(image.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)     

        blob = color_threshold(image_np)
        img_remove_noise = remove_noise(blob)

        sign, sign_blob = circle_contour(image_np, img_remove_noise)

        sign_label = None

        if sign is not None:
            sign_label = hu_moments_classify(sign)
            x, y, w, h = get_boundingbox(sign_blob)

            if not sign_label is None:
                sign_info.sign_label = sign_label
                sign_info.x = x
                sign_info.y = y
                sign_info.width = w
                sign_info.height = h

                pub.publish(sign_info)
            else:
                sign_info.sign_label = -1
                pub.publish(sign_info)
        else:
            sign_info.sign_label = -1
            pub.publish(sign_info)
            
if __name__ == '__main__':
    listener()
