# Aruco_Ros2
=========

[![pipeline status](https://gitlab.com/raymondchaneee/aruco_ros2/badges/master/pipeline.svg)](https://gitlab.com/raymondchaneee/aruco_ros2/commits/master)
[![License](https://img.shields.io/badge/License-BSD%202--Clause-orange.svg)](https://opensource.org/licenses/BSD-2-Clause)
[![codecov](https://codecov.io/gl/raymondchaneee/aruco_ros2/branch/master/graph/badge.svg)](https://codecov.io/gl/raymondchaneee/aruco_ros2)
[![Documentation Status](https://readthedocs.org/projects/aruco-ros2/badge/?version=latest)](https://aruco-ros2.readthedocs.io/en/latest/)

Software package and ROS2 wrappers of the [Aruco][1] Augmented Reality marker detector library.


### Features
<!-- <img align="right" src="https://raw.github.com/pal-robotics/aruco_ros/master/aruco_ros/etc/marker_in_hand.jpg" /> -->

 * High-framerate tracking of AR markers
 
 * Generate AR markers with given size and optimized for minimal perceptive ambiguity (when there are more markers to track)
 
 * Enhanced precision tracking by using boards of markers
 
 * ROS2  wrappers


### Applications

 * Object pose estimation
 * Visual servoing: track object and hand at the same time

<!-- <img align="right" src="https://raw.github.com/pal-robotics/aruco_ros/master/aruco_ros/etc/reem_gazebo_floating_marker_world.png"/> -->


### ROS2 API

### Build
 * direct to workspace `src` , then git clone the repo
```
cd <path/to/workspace/>/src
git clone https://gitlab.com/raymondchaneee/aruco_ros2.git
```
 * direct to workspace then, install dependency using `rosdep`, then build using `colcon`
```
 cd <path/to/workspace>
 rosdep install --from-paths src --ignore-src --rosdistro eloquent -y
 colcon build 
```
 * if build successful, then run test to check error whether is pass (except for `copyright` issue due to eloquent ament_lint did not update its ament_copyright repo. which is not a problem to run the whole repo)
```
   colcon test --pakages-select aruco_ros2
   colcon test-result --all
```
### Generate markers

`ros2 run aruco optimalmarkers <quantity of marker> <name of marker> <marker number> <min marker number, 30 minimum>` e.g.
```
ros2 run aruco optimalmarkers 1 testmark 360 30
```
#### Messages

 * aruco_ros/Marker.msg

        std_msg/Header header
        uint32 id
        geometry_msgs/PoseWithCovariance pose
        float64 confidence

 * aruco_ros/MarkerArray.msg

        std_msg/Header header
        aruco_ros/Marker[] markers

### Test 

 * to test run please install a camera pkg example, `ros2_usb_camera`, and do 

	```
 	ros2 run usb_camera_driver usb_camera_driver_node 
	```
	with input camera calibration parameter in .yaml file


 * then execute

	```
 	ros2 launch aruco_ros2 single.launch.xml 
	```
 	There will be a display of camera showing, put a generated aruco marker, will shows the detection of marker on window.

 * Start the `marker_publisher` node which will start tracking the multiple marker and will publish its pose in the camera frame in `markers` topic, please take note that this application will not have any visualization in rviz, only result image. 
 
    ```
    roslaunch aruco_ros marker_publisher.launch.xml
    ```



[1]: http://www.sciencedirect.com/science/article/pii/S0031320314000235 "Automatic generation and detection of highly reliable fiducial markers under occlusion by S. Garrido-Jurado and R. Muñoz-Salinas and F.J. Madrid-Cuevas and M.J. Marín-Jiménez 2014"
