cmake_minimum_required(VERSION 3.5)
project(aruco_description)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Install files
install(DIRECTORY 
        launch 
        meshes 
        rviz 
        urdf
        scripts
        DESTINATION share/${PROJECT_NAME})


# Specify the install destination for executables
#install(
#    DIRECTORY scripts/
#    DESTINATION lib/${PROJECT_NAME}
#    FILE_PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ GROUP_EXECUTE GROUP_READ WORLD_EXECUTE WORLD_READ
#)

# Set the path to the Python script
set(GENERATE_ARUCO_MESH_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/scripts/generate_aruco_mesh.py")

# Install the Python script with executable permissions
install(PROGRAMS ${GENERATE_ARUCO_MESH_SCRIPT}
        DESTINATION lib/${PROJECT_NAME}
)

# Set the executable permissions explicitly
install(CODE "execute_process(COMMAND chmod +x ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/generate_aruco_mesh.py)")



ament_package()