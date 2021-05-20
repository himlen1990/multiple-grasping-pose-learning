cmake_minimum_required(VERSION 3.0.2)
project(multiple_grasping_pose_learning)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  pcl_ros
  tf
  roscpp
  rospy
  geometry_msgs
  image_transport
  )

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

include_directories(${catkin_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})



add_service_files(
  FILES
  getobjectpose.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES multiple-grasping-pose-learning
CATKIN_DEPENDS roscpp message_runtime
)


add_executable(data_collection src/data_collection.cpp)
target_link_libraries(data_collection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
