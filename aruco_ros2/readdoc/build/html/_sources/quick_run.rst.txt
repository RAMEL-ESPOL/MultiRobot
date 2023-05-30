.. _api_Quick_run:

Quick run
=========

1) Generate markers 
^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

   ros2 run aruco optimalmarkers <quantity of marker> <name of marker> <marker number> <min marker number, 30 minimum> e.g.
   ros2 run aruco optimalmarkers 1 testmark 360 30

2) Test 
^^^^^^^

to test run please install a camera pkg example, ros2_usb_camera, and do

.. code-block:: bash

   ros2 run usb_camera_driver usb_camera_driver_node 

with input camera calibration parameter in .yaml file


then execute

.. code-block:: bash

   ros2 launch aruco_ros2 single.launch.xml 

There will be a display of camera showing, put a generated aruco marker, will shows the detection of marker on window.


Start the marker_publisher node which will start tracking the multiple marker and will publish its pose in the camera frame in markers topic, please take note that this application will not have any visualization in rviz, only result image.

.. code-block:: bash

   roslaunch aruco_ros marker_publisher.launch.xml