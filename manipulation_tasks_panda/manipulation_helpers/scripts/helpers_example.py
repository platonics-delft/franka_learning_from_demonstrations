from geometry_msgs.msg import Pose
from manipulation_helpers.pose_transform_functions import orientation_2_quaternion, pose_2_transformation
import manipulation_helpers


if __name__ == "__main__":
    pose = Pose()
    pose.orientation.w = 1
    print(orientation_2_quaternion(pose.orientation))
    print(pose_2_transformation(pose))

