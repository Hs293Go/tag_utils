cmake_minimum_required(VERSION 2.8.3)
project(tag_utils)


add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  camera_info_manager
  cv_bridge
  geometry_msgs
  image_transport
  nav_msgs
  roscpp
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)

catkin_package()


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GSTREAMER_INCLUDE_DIRS}
  include
)

add_executable(camera_node src/camera_node.cpp)

add_executable(transform_backend src/transform_backend.cpp)

target_link_libraries(camera_node
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${catkin_LIBRARIES}
  fmt::fmt
)

target_link_libraries(transform_backend
  ${OpenCV_LIBS}
  ${EIGEN3_LIBS}
  ${catkin_LIBRARIES}
)