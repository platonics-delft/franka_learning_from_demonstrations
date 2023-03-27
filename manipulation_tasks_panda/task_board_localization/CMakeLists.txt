cmake_minimum_required(VERSION 3.5)
project(task_board_localization)

find_package(catkin REQUIRED COMPONENTS roscpp pcl_ros sensor_msgs rospy std_msgs geometry_msgs std_msgs cmake_modules eigen_conversions message_generation roslib)
find_package(PCL 1.2 REQUIRED)


add_service_files(FILES 
                  PointDetect.srv
                  )


generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)


catkin_package(
  # INCLUDE_DIRS 
  #LIBRARIES 
  #CATKIN_DEPENDS cmake_modules eigen_conversions geometry_msgs message_runtime roscpp sensor_msgs std_msgs  
  #DEPENDS 
)




