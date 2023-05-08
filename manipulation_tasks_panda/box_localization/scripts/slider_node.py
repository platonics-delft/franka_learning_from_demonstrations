#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32, Float32MultiArray
from std_srvs.srv import Trigger, TriggerResponse
from cv_bridge import CvBridge
import cv2
import numpy as np
from scipy.ndimage import gaussian_filter1d
from scipy import signal
from sklearn.cluster import KMeans
from sklearn.mixture import GaussianMixture
from scipy.signal import find_peaks
class SliderNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node("slider_node")

        self.status = Bool(data=False)
        self.result = Float32MultiArray(data=[0, 1e4, 0.0])
        self.detection_treshold=100
        self.rate = rospy.Rate(20)
        self.running = False
        self.consecutive_detections = 0

        # Subscribe to original image topic
        rospy.Subscriber("/camera/color/image_raw", Image, self.image_callback)

        # Create publisher for filtered image topic
        self.filtered_image_pub = rospy.Publisher(
            "slider_node/filtered_image", Image, queue_size=10
        )

        # Create publisher for cropped image topic
        self.cropped_image_pub = rospy.Publisher(
            "slider_node/cropped_image", Image, queue_size=10
        )

        # Create publisher for detection status
        self.status_pub = rospy.Publisher("slider_node/detected", Bool, queue_size=10)

        # Create publisher for delta direction that the slider needs to go
        self.result_pub = rospy.Publisher("slider_node/result", Float32MultiArray, queue_size=10)

        # Create service to trigger the detection
        self.service = rospy.Service('slider_detection', Trigger, self.trigger_detection)

    def trigger_detection(self, req):
        print('detecting')
        self.consecutive_detections = 0
        self.running = True
        resp = TriggerResponse()
        resp.message = "Succesfully triggered detection of green triangle"
        resp.success = True
        return resp

    def image_callback(self, msg):
        if not self.running: return

        # Convert ROS Image message to OpenCV image
        bridge = CvBridge()
        img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        ## Step 1: Blue color filter
        lower_blue = np.array([80, 150, 150])
        upper_blue = np.array([120, 255, 255])

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        filtered_img = cv2.bitwise_and(img, img, mask=mask)

        ## Step 2: Find largest rectangle contour
        all_contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        all_contours_area = [cv2.contourArea(c) for c in all_contours]
        if len(all_contours_area) == 0: return

        contour = all_contours[np.argmax(all_contours_area)]
        rect = cv2.minAreaRect(contour)

        # Apply the rotation to the image
        #print(rect)
        w = abs(rect[0][0]-rect[1][0])
        h = abs(rect[0][1] - rect[1][1])
        angle = rect[2]
        if abs(angle - 90) < 40:
            angle -= 90
        M = cv2.getRotationMatrix2D((rect[0][0]+w//2, rect[0][1]+h//2), angle, 1.0)
        filtered_img = cv2.warpAffine(filtered_img, M, filtered_img.shape[1::-1], flags=cv2.INTER_LINEAR)
        mask = cv2.warpAffine(mask, M, mask.shape[1::-1], flags=cv2.INTER_LINEAR)
        img = cv2.warpAffine(img, M, img.shape[1::-1], flags=cv2.INTER_LINEAR)

        # optionally draw
        all_contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        all_contours_area = [cv2.contourArea(c) for c in all_contours]
        if len(all_contours_area) == 0: return

        contour = all_contours[np.argmax(all_contours_area)]
        cv2.drawContours(filtered_img, [contour], 0, (0, 0, 255), 2)
        x,y,w,h = cv2.boundingRect(contour)
        cv2.rectangle(filtered_img, (x, y), (x + w, y + h), (0,255,0), 4)
        rect = cv2.minAreaRect(contour)

        ## Step 3: Crop to slider line
        rect = cv2.boundingRect(contour)
        x, y, w, h = rect
        cropped_img = img[y - 50 : y - 10, x + 10 : x + w - 10]
        h, w, _ = cropped_img.shape

        # publish intermediate img
        filtered_image_msg = bridge.cv2_to_imgmsg(filtered_img, encoding="bgr8")
        self.filtered_image_pub.publish(filtered_image_msg)

        if (
            np.mean(cropped_img[: h // 2, :, 0]) < 180
            or h == 0 or w == 0
        ):
            self.status.data = False
            self.result.data[2] = 0.0
            print(f"Mean red bot {np.mean(cropped_img[h // 2 :, :, 2])}")
            print(f"Mean red top {np.mean(cropped_img[: h // 2, :, 2])}")
            print(f"Mean blue bot {np.mean(cropped_img[h // 2 :, :, 0])}")
            print(f"Mean blue top {np.mean(cropped_img[: h // 2, :, 0])}")
            print("stopping early")
            return
        else:
            self.status.data = True

        ## Step 4: Bottom arrow detection (red)
        mean_reds = np.mean(cropped_img[3 * h // 4 :, :, 2], axis=0)
        idx_bot = mean_reds.argmax()

        ## Step 5: First top arrow detection (white)
        mean_green_minus_red = np.mean(cropped_img[: h // 2, :, 1], axis=0) - np.mean(cropped_img[: h // 2, :, 2], axis=0)
        mean_green_minus_red = gaussian_filter1d(mean_green_minus_red, sigma=10)

        window_size = 5
        mean_green_minus_red_smooth = np.convolve(mean_green_minus_red, np.ones(window_size)/window_size, mode='valid')

        triangle_position=np.argmax(mean_green_minus_red_smooth)
        if (np.max(mean_green_minus_red_smooth)>self.detection_treshold) and mean_green_minus_red_smooth[triangle_position] > 80:
            cropped_img[: h // 4,  triangle_position] = [0, 255, 0]
            self.result.data[1] = idx_bot - triangle_position

            cropped_img[3 * h // 4 , idx_bot] = [0, 255, 0]

            cropped_image_msg = bridge.cv2_to_imgmsg(cropped_img, encoding="bgr8")
            self.cropped_image_pub.publish(cropped_image_msg)

            self.consecutive_detections += 1
        else:
            self.consecutive_detections = 0

    def run(self):
        while not rospy.is_shutdown():
            if self.running and (self.consecutive_detections > 3):
                self.status_pub.publish(self.status)
                self.result_pub.publish(self.result)

            self.rate.sleep()


if __name__ == "__main__":
    node = SliderNode()
    node.run()
