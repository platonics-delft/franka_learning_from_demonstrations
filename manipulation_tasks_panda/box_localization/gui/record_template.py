#!/bin/python3

import cv2
import sys
import rospy
import os
import yaml
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import os
import rospkg

class Template():
    def __init__(self):
        rospy.init_node('Template_recording')
        self.params = dict()
    def crop_image(self, event, x, y, flags, param):
        global mouseX, mouseY, cropping
        if event == cv2.EVENT_LBUTTONDOWN:
            mouseX, mouseY = x, y
            cropping = True
        elif event == cv2.EVENT_LBUTTONUP:
            mouseX2, mouseY2 = x, y
            cropping = False
            cv2.imshow("image", self.image)
            y_low = min(mouseY, mouseY2)
            y_high = max(mouseY, mouseY2)
            x_low = min(mouseX, mouseX2)
            x_high = max(mouseX, mouseX2)
            self.params['crop'] = [x_low, x_high, y_low, y_high]
            print(f'{x_low}, {x_high}, {y_low}, {y_high}')

            cropped_image = self.image[y_low: y_high, x_low:x_high]
            cv2.imshow("cropped", cropped_image)
            cv2.imwrite(f"{self.save_dir}/template.png", cropped_image)
    
    def record(self, name='template_test'):
         
            rospack = rospkg.RosPack()
            box_localization_path=rospack.get_path('box_localization')
            self.save_dir = box_localization_path +f"/cfg/{name}"
            self.params['template_path'] = f"/cfg/{name}"+"/full_image.png"
            os.mkdir(self.save_dir)

            try:
                bridge = CvBridge()
                msg_depth = rospy.wait_for_message('camera/depth/image_rect_raw', Image, timeout=10)
                msg = rospy.wait_for_message('camera/color/image_raw', Image, timeout=10)
                self.image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                depth = bridge.imgmsg_to_cv2(msg_depth, desired_encoding="passthrough")
                cv2.imwrite(f"{self.save_dir}/full_image.png", self.image)
                cv2.imwrite(f"{self.save_dir}/depth.png", depth)
            except:
                print("Image is not found after timeout of 10 seconds")

            print("Click and drag to select template")
            print("Press 'q' to quit")
            # Create window and set mouse callback function
            cv2.namedWindow("image")
            cv2.setMouseCallback("image", self.crop_image)

            # Loop until user presses 'q'
            cropping = False
            while True:
                cv2.imshow(f"image", self.image)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            
            depth_row=depth[self.params['crop'][2]:self.params['crop'][3], self.params['crop'][0]:self.params['crop'][1]].reshape(-1)
            print(depth_row)
            self.params['depth'] = float(np.median(depth_row))
            with open(f"{self.save_dir}/params.yaml", 'w') as file:
                yaml.dump(self.params, file)


