import rospy
from object_localization.pointcloud_saver import PointCloudSaver

if __name__ == "__main__":
    try:
        PointCloudSaver()
    except rospy.ROSInterruptException:
        pass

