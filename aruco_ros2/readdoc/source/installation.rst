.. _api_Installation_Guide:

Installation Guide
==================

1. clone repo
^^^^^^^^^^^^^
direct to workspace `src` , then git clone the repo

.. code-block:: bash

   cd <path/to/workspace/>/src
   git clone https://gitlab.com/raymondchaneee/aruco_ros2.git

2.install dependency and build
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
direct to workspace then, install dependency using `rosdep`, then build using `colcon`

.. code-block:: bash

   cd <path/to/workspace>
   rosdep install --from-paths src --ignore-src --rosdistro eloquent -y
   colcon build 

3. Test the repo
^^^^^^^^^^^^^^^^
if build successful, then run test to check error whether is pass (except for `copyright` issue due to eloquent ament_lint did not update its ament_copyright repo. which is not a problem to run the whole repo)

.. code-block:: bash

   colcon test --pakages-select aruco_ros2
   colcon test-result --all

