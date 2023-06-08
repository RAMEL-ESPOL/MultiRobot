cmake_minimum_required(VERSION 3.5)
project(aruco_ros2)

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

# find dependencies
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
####
add_library(aruco_ros_utils src/aruco_ros_utils.cpp)
#target_link_libraries(aruco_ros_utils ${catkin_LIBRARIES})

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


add_executable(single  node/simple_single_node.cpp
                      src/simple_single.cpp
                      src/aruco_ros_utils.cpp)
#add_dependencies(single ${PROJECT_NAME}_gencfg)
target_link_libraries(single ${OpenCV_LIBRARIES})
ament_target_dependencies(single ${dependencies})


add_executable(marker_publisher node/marker_publish_node.cpp
                                src/marker_publish.cpp
                                src/aruco_ros_utils.cpp)
#add_dependencies(marker_publisher ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
target_link_libraries(marker_publisher ${OpenCV_LIBRARIES})
ament_target_dependencies(marker_publisher ${dependencies})





#############
## Install ##
#############

install(TARGETS single aruco_ros_utils marker_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
)

install(TARGETS
    single
    marker_publisher
    DESTINATION lib/${PROJECT_NAME}
)

foreach(dir etc launch src)
    install(DIRECTORY ${dir}/
        DESTINATION  share/${PROJECT_NAME}/${dir})
endforeach()


#############
## GTest ##
#############
# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # uncomment the line when a copyright and license is not present in all source files
#   #set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # uncomment the line when this package is not in a git repo
#   #set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()


if(BUILD_TESTING)

  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # Excluding to avoid conflicting Doxygen python code blocks.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_pep257
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()


  ament_add_gtest(test_markerpublisher_aruco_ros2 test/test_markerpublisher_aruco_ros2.cpp
    src/marker_publish.cpp
    src/aruco_ros_utils.cpp)
    ament_target_dependencies(test_markerpublisher_aruco_ros2   ${dependencies})
    target_link_libraries(test_markerpublisher_aruco_ros2  ${OpenCV_LIBRARIES})

  ament_add_gtest(test_simplesingle_aruco_ros2 test/test_simplesingle_aruco_ros2.cpp
    src/simple_single.cpp
    src/aruco_ros_utils.cpp)
    ament_target_dependencies(test_simplesingle_aruco_ros2   ${dependencies})
    target_link_libraries(test_simplesingle_aruco_ros2  ${OpenCV_LIBRARIES})

  # ament_add_gtest(test_aruco_function test/test_aruco_function.cpp
  #   src/marker_publish.cpp
  #   src/aruco_ros_utils.cpp)
  # #autoware_set_compile_options(test_aruco_function)
  # ament_target_dependencies(test_aruco_function   ${dependencies})
  # target_link_libraries(test_aruco_function  ${OpenCV_LIBRARIES})

endif()


ament_package()
