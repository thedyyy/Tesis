# -*- coding: utf-8 -*-

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy   
import os

bridge = CvBridge()
bag = rosbag.Bag('/home/benjamin/Desktop/rosbag_chica/2023-12-04-15-18-58.bag')

# Crear un directorio para guardar las imágenes si no existe
output_dir = "/home/benjamin/Desktop/rosbag_chica/8bits"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

image_counter = 0
print("Reading from rosbag...")
for topic, msg, t in bag.read_messages(topics=['/a65/thermal_camera/imth']):
    print("Found a message on topic {}. Message type is {}".format(topic, type(msg)))
    try:
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="mono16")
        imgout = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
        imgout = cv2.applyColorMap(imgout, cv2.COLORMAP_JET)  # Aplicar el mapa de colores
        # Guardar la imagen
        image_filename = os.path.join(output_dir, "image_{}.png".format(image_counter))
        cv2.imwrite(image_filename, imgout)
        print("Saved image to {}".format(image_filename))
        image_counter += 1
    except CvBridgeError as e:
        print(e)

bag.close()
