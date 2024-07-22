import rospy
from sensor_msgs.msg import Image

from camera_feedback import image_process
from cv_bridge import CvBridgeError, CvBridge


class CameraUtilsNode():
    _processed_image: Image
    _ds_factor: int = 4
    _row_crop_top: float = 0.0
    _row_crop_bottom: float = 0.3
    _row_crop_left: float = 0.4
    _row_crop_right: float = 1.7

    def __init__(self):
        rospy.init_node('camera_utils_node')
        self.bridge = CvBridge()
        self._rate = rospy.Rate(10)
        self._processed_image = Image()
        self.establish_ros_connections()

    def establish_ros_connections(self):
        self._image_publisher = rospy.Publisher('processed_image', Image, queue_size=10)
        self._image_subscriber = rospy.Subscriber('/camera/color/image_raw', Image, self._image_callback)

    def _image_callback(self, image: Image):
        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        self._processed_image = image_process(
            cv_image,
            self._ds_factor,
            self._row_crop_top,
            self._row_crop_bottom,
            self._row_crop_left,
            self._row_crop_right,
        )


    def run(self):
        while not rospy.is_shutdown():
            self._rate.sleep()
            image_msg = self.bridge.cv2_to_imgmsg(self._processed_image)
            self._image_publisher.publish(image_msg)

if __name__ == '__main__':
    node = CameraUtilsNode()
    node.run()
