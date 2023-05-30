cmake_minimum_required(VERSION 3.5)
project(aruco)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_ros REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rmw_implementation REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
#find_package(rosidl_default_generators REQUIRED)


find_package(OpenCV REQUIRED)

#catkin_package(
##  INCLUDE_DIRS include
#  LIBRARIES aruco
#)

include_directories(
  include

  ${OpenCV_INCLUDE_DIRS}
)

#ament_target_dependencies( aruco  "OpenCV")
add_library(aruco SHARED
  src/aruco/arucofidmarkers.cpp
  src/aruco/cvdrawingutils.cpp
  src/aruco/cameraparameters.cpp
  src/aruco/board.cpp
  src/aruco/marker.cpp
  src/aruco/boarddetector.cpp
  src/aruco/markerdetector.cpp
)
target_link_libraries(aruco
   ${OpenCV_LIBRARIES})

add_executable(optimalmarkers
  src/aruco/aruco_selectoptimalmarkers.cpp)


ament_target_dependencies(optimalmarkers
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "rmw_implementation"
)
target_link_libraries(optimalmarkers aruco ${OpenCV_LIBRARIES})

#############
## Install ##
#############

install(TARGETS aruco optimalmarkers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
   DESTINATION include
   FILES_MATCHING PATTERN "*.h"
)

install(TARGETS
    optimalmarkers
    DESTINATION lib/${PROJECT_NAME}
)
ament_export_include_directories(include)
ament_export_libraries(aruco)
ament_package()

