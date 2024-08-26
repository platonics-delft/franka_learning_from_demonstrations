cmake_minimum_required(VERSION 3.0.2)
project(object_localization)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  sensor_msgs
  std_msgs
  message_generation
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  ComputeLocalization.srv
  GlobalRegistrationLocalizer.srv
  IterativeRegistrationLocalizer.srv
  SavingPointcloud.srv
)

# Make sure that your Python modules are available for import within your package
catkin_python_setup()

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
  sensor_msgs
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install the box localization package, including Python modules
install(DIRECTORY object_localization
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS
  scripts/static_transform_camera.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
