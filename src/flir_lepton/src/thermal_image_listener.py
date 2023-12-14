#!/usr/bin/env python

import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

counter = 0
dir = "/home/benjamin/Desktop/imagenes_termales/"

def image_callback(msg):
    global counter
    try:
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="mono16")
        imgout = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        width, height = imgout.shape #, channels
        resize_factor = 4
        #imgout = cv2.resize(imgout, (width*resize_factor, height*resize_factor))
        #rospy.loginfo("Width: %d", width)
        #rospy.loginfo("Height: %d", height)
        #rospy.loginfo("Number of Channels: %d", channels)
        imgout = cv2.applyColorMap(imgout, cv2.COLORMAP_JET)
        cv2.imshow("ThermalImage", imgout)
        filename = dir + str(counter) + ".jpg"
        cv2.imwrite(filename, imgout)
        counter += 1
        cv2.waitKey(30)
    except CvBridgeError as e:
        rospy.logerr("Could not convert from '%s' to 'mono16'. %s", msg.encoding, str(e))

def thermal_image_listener():
    rospy.init_node("thermal_image_listener")
    rospy.loginfo("Thermal Image Listener node has been initialized.")
    cv2.namedWindow("ThermalImage")
    cv2.startWindowThread()
    it = CvBridge()
    #sub = rospy.Subscriber("thermal_camera/image", Image, image_callback)
    sub = rospy.Subscriber("/thermal_camera/image", Image, image_callback)
    rospy.spin()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        thermal_image_listener()
    except rospy.ROSInterruptException:
        pass

