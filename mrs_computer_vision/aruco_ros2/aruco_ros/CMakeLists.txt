cmake_minimum_required(VERSION 3.5)
project(aruco_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(aruco REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(aruco_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)

include_directories(
  include
)
include_directories(
  SYSTEM

  ${OpenCV_INCLUDE_DIRS}
)

message(${sensor_msgs_INCLUDE_DIRS})

#generate_dynamic_reconfigure_options(
#  cfg/ArucoThreshold.cfg
#)

add_library(aruco_ros_utils src/aruco_ros_utils.cpp)

#target_include_directories(aruco_ros_utils
#  PUBLIC
#  include)

#target_include_directories(aruco_ros_utils
#  SYSTEM PUBLIC
#  ${OpenCV_INCLUDE_DIRS}
#)

ament_target_dependencies(aruco_ros_utils
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "rmw_implementation"
  "aruco"
  "tf2_msgs"
  "tf2"
  "tf2_geometry_msgs"
  "geometry_msgs"
  "aruco_msgs"
  "visualization_msgs"
  "rclcpp_components"
  "OpenCV"
  "tf2_ros"
)

set(dependencies
  cv_bridge
  geometry_msgs
  image_transport
  sensor_msgs
  visualization_msgs
  rclcpp
  rclcpp_components
  tf2_msgs
  aruco
  OpenCV
  aruco_msgs
  tf2_ros
  tf2_geometry_msgs
  tf2
)

#add_executable(single src/simple_single.cpp
#                      src/aruco_ros_utils.cpp)

#target_include_directories(single
#  PUBLIC
#  include)

#target_include_directories(single
#  SYSTEM PUBLIC
#  ${OpenCV_INCLUDE_DIRS}
#)

#ament_target_dependencies(single ${dependencies})
#target_link_libraries(single ${OpenCV_LIBRARIES})

add_executable(marker_publisher src/marker_publish.cpp
                                src/aruco_ros_utils.cpp)

#target_include_directories(marker_publisher
#  PUBLIC
#  include)

#target_include_directories(marker_publisher
#  SYSTEM PUBLIC
#  ${OpenCV_INCLUDE_DIRS}
#)

ament_target_dependencies(marker_publisher ${dependencies})
target_link_libraries(marker_publisher ${OpenCV_LIBRARIES})


#############
## Install ##
#############

install(TARGETS aruco_ros_utils marker_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS marker_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
        FILES_MATCHING PATTERN "*.h"
)

foreach(dir etc launch src)
    install(DIRECTORY ${dir}/
            DESTINATION share/${PROJECT_NAME}/${dir})
endforeach()

ament_package()


