#!/usr/bin/env python

import rospy
import tf

def load_camera_transform():
    position = rospy.get_param("/camera_transform/position", {})
    orientation = rospy.get_param("/camera_transform/orientation", {})
    translation = (position['x'], position['y'], position['z'])
    rotation = (orientation['x'], orientation['y'], orientation['z'], orientation['w'])
    print("Loaded camera transform: translation: {}, rotation: {}".format(position, orientation))
    return translation, rotation
def static_transform_publisher():
    rospy.init_node('static_transform_publisher')
    
    static_broadcaster = tf.TransformBroadcaster()
    
    transform_data = load_camera_transform()
    if transform_data is None:
        rospy.logerr("Failed to load camera transform data.")
        return

    translation, rotation = transform_data
    
    rate = rospy.Rate(100)  # Rate for sending transforms, adjust as needed

    while not rospy.is_shutdown():
        static_broadcaster.sendTransform(
            translation,
            rotation,
            rospy.Time.now(),
            "camera_color_optical_frame",
            "panda_hand"
        )
        rate.sleep()

if __name__ == '__main__':
    try:
        static_transform_publisher()
    except rospy.ROSInterruptException:
        pass
