# -*- coding: utf-8 -*-

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy   
import os

bridge = CvBridge()
name= "_2018-10-05-09-37-48_0.bag"
bag = rosbag.Bag('/home/benjamin/Desktop/rosbags_db_2018/'+name)

# Crear un directorio para guardar las imágenes si no existe
output_dir = "/home/benjamin/Desktop/dataset_18/"+name[:-4]
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

image_counter = 0
print("Reading from rosbag...")
for topic, msg, t in bag.read_messages(topics=['/thermal_camera/image']):
    print("Found a message on topic {}. Message type is {}".format(topic, type(msg)))
    try:
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        # Guardar la imagen
        image_filename = os.path.join(output_dir, "image_{}.png".format(image_counter))
        cv2.imwrite(image_filename, img)
        print("Saved image to {}".format(image_filename))
        image_counter += 1
    except CvBridgeError as e:
        print(e)

bag.close()
