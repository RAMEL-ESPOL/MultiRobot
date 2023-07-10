cmake_minimum_required(VERSION 3.5)
project(realsense2_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Add the package's include directories
include_directories(include)

# Create the lib directory during the build
#file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/lib)

# Install Python scripts
#ament_python_install_package(src)

# Install Python files to lib
#install(DIRECTORY src/
#  DESTINATION lib/${PROJECT_NAME}
#  FILES_MATCHING PATTERN "*.py"
#)

# Set executable permissions for multiple files
#set(EXECUTABLE_FILES
#  l515_spawn.py
  # Add more files here as needed
#)

#foreach(file ${EXECUTABLE_FILES})
#  set(EXECUTABLE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/realsense2_description/${file})
#  execute_process(
#    COMMAND ${CMAKE_COMMAND} -E chmod +x ${EXECUTABLE_PATH}
#  )
#endforeach()

install(PROGRAMS  
  scripts/l515_spawn.py  
  DESTINATION lib/${PROJECT_NAME} )


# Set compiler flags if necessary
# target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra)

# Add dependencies, if any
# find_package(example_dependency REQUIRED)
# target_link_libraries(${PROJECT_NAME} example_dependency)

# Install other package files if needed
install(DIRECTORY 
        launch 
        meshes 
        rviz 
        urdf
        worlds
        scripts
        DESTINATION share/${PROJECT_NAME}
)

# Set up the ament package
ament_export_dependencies(rclcpp)
ament_package()
