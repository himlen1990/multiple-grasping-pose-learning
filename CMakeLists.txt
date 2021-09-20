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
  image_geometry
  std_srvs
  )

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

add_service_files(
  FILES
  gettablepose.srv
  graspanobject.srv
  objectdetect.srv
  grasppredict.srv
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES multiple-grasping-pose-learning
CATKIN_DEPENDS roscpp message_runtime
)


add_executable(data_collection_turntable src/data_collection_turntable.cpp)
target_link_libraries(data_collection_turntable ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(deploy src/deploy.cpp)
target_link_libraries(deploy ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

add_executable(get_point_position utils/get_point_position.cpp)
target_link_libraries(get_point_position ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})
