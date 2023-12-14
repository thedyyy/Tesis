#8 to 16 bits 

import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy   
import os

bridge = CvBridge()
bag = rosbag.Bag('/home/benjamin/Desktop/rosbagfolder/2023-11-02-17-16-23.bag')

# Crear un directorio para guardar las imágenes si no existe
output_dir = "/home/benjamin/Desktop/rosbagfolder/8bits"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

image_counter = 0
for topic, msg, t in bag.read_messages(topics=['/thermal_camera/image']):
    if isinstance(msg, Image):
        try:
            img = bridge.imgmsg_to_cv2(msg, desired_encoding="mono16")
            imgout = cv2.normalize(img, None, 0, 255, cv2.NORM_MINMAX, cv2.CV_8U)
            # Guardar la imagen
            image_filename = os.path.join(output_dir, f"image_{image_counter}.png")
            cv2.imwrite(image_filename, imgout)
            image_counter += 1
        except CvBridgeError as e:
            print(e)

bag.close()