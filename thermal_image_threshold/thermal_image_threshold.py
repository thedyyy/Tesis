#!/usr/bin/env python

import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

pub = None
pub2 = None
msg_out_bin = None
msg_out_gray = None

Threshold2Zero = 0
Threshold2ZeroInverted = 1

trackbar_min_value = "Min Value"
trackbar_max_value = "Max Value"
window_name = "ThermalImage"

min_bar_value = 0 #+1730
max_bar_value = 70000 #-10000+7780
d_value = max_bar_value - min_bar_value

threshold_min_value = 0
threshold_max_value = d_value

threshold_min_value_ = threshold_min_value + min_bar_value
threshold_max_value_ = threshold_max_value + min_bar_value
threshold_bin_value_ = d_value/2 + min_bar_value

morph_elem = 2  # elipse
morph_size = 1  # porte del filtro
operation = 2  # opening
element = cv2.getStructuringElement(morph_elem, (2 * morph_size + 1, 2 * morph_size + 1), (morph_size, morph_size))

def on_track_bar_min(value):
    global threshold_min_value_
    threshold_min_value_ = value + min_bar_value

def on_track_bar_max(value):
    global threshold_max_value_
    threshold_max_value_ = value + min_bar_value

def on_track_bar_bin(value):
    global threshold_bin_value_
    threshold_bin_value_ = value + min_bar_value

def LoopPixels(img):
    global threshold_min_value_, threshold_max_value_, threshold_bin_value_

    img_gray = np.where((img<threshold_min_value_)|(img>threshold_max_value_),0,img)
    img_bin = img_gray.copy()
    img_gray = cv2.normalize(img_gray, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)

    _, img_bin = cv2.threshold(img_bin, threshold_bin_value_, 255, cv2.THRESH_BINARY)
    img_bin = cv2.normalize(img_bin, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)




    return img_bin, img_gray

def imageCallback(msg_in):
    global pub, pub2, msg_out_bin, msg_out_gray, window_name

    try:
        bridge = CvBridge()
        image = bridge.imgmsg_to_cv2(msg_in, desired_encoding="mono16")
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(image)
        print(min_val, max_val, threshold_min_value_, threshold_max_value_)
        bin_img, gray_img = LoopPixels(image)


        if gray_img is not None:
            msg_out_bin = bridge.cv2_to_imgmsg(bin_img, encoding="mono8")
            pub.publish(msg_out_bin)

            msg_out_gray = bridge.cv2_to_imgmsg(gray_img, encoding="mono8")
            pub2.publish(msg_out_gray)

            #gray_img_resized = cv2.resize(gray_img, None, fx=1, fy=1, interpolation=cv2.INTER_CUBIC)
            #bin_img_resized = cv2.resize(bin_img, None, fx=1, fy=1, interpolation=cv2.INTER_CUBIC)

            cv2.imshow(window_name, gray_img)
            cv2.waitKey(1)
    except CvBridgeError as e:
        rospy.logerr(e)

def main():
    global pub, pub2, window_name, threshold_min_value, d_value, threshold_max_value

    cv2.namedWindow(window_name)
    cv2.createTrackbar('trackbarmin', window_name, 0, d_value, on_track_bar_min)
    cv2.createTrackbar('trackbarmax', window_name, d_value, d_value, on_track_bar_max)
    cv2.createTrackbar('trackbarbin', window_name, d_value/2, d_value, on_track_bar_bin)

    #Threshold_Demo(0)

    rospy.init_node('thermal_image_threshold', anonymous=True)
    pub = rospy.Publisher('a65/thermal_camera/bin', Image, queue_size=1)
    pub2 = rospy.Publisher('a65/thermal_camera/imth', Image, queue_size=1)
    rospy.Subscriber('/thermal_camera/image', Image, imageCallback)

    rospy.spin()
    cv2.destroyWindow(window_name)

if __name__ == '__main__':
    main()
