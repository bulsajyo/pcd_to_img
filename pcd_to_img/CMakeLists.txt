cmake_minimum_required(VERSION 3.0.2)
project(pcd_to_img)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  pcl_ros
  pcl_conversions
  roscpp
  sensor_msgs
  std_msgs
  geometry_msgs
  cv_bridge
)

set(SRC_FILES
  src/pcd_to_img_main.cpp
  src/projection_IP.cpp
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(PCL 1.10 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS 
    PCL
)

include_directories(
  include 
  SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
link_directories(
  ${PCL_LIBRARY_DIRS}
)
add_definitions(
  ${PCL_DEFINITIONS}
)
add_library(${PROJECT_NAME}
  ${SRC_FILES}
)

add_executable(${PROJECT_NAME}_node
  src/pcd_to_img_main.cpp
)

target_link_libraries(${PROJECT_NAME}_node 
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)