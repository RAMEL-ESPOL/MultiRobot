# Aruco Description

## Overview

This is a simple package that helps the creation of a custom Aruco code marker with the desired ID that can be spawned in a custom gazebo simulation but can also be loaded into a ROS parameter and viewed from rviz.

## License

The source code is released under a [BSD 3-Clause license](LICENSE).

**Author: Valerio Magnago<br />
Maintainer: Valerio Magnago, valerio.magnago@hotmail.it**

The PACKAGE Aruco Description package has been tested under [ROS] Melodic and Ubuntu 18.04.

## Installation

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using

```sh
cd catkin_workspace/src
git clone https://github.com/ValerioMa/aruco_description
cd ../
rosdep install --from-paths . --ignore-src
catkin_make
source devel/setup.bash
```

## Usage

Suppose that you want to use the description of the Aruco code with id=*1*  from the *DICT_4X4_50* dictionary.
The first step is to generate the mesh of that specific code:

```sh
roslaunch aruco_description generate_aruco.launch aruco_dictionary:="DICT_4X4_50" aruco_ids:="1"
```

The launch file accepts as parameters the dictionary and the ids to generate. See the launch parameters description below to have more details on them.

Once the aruco is generate we can:

- spawn the code in a running gazebo world. For this a commodity launch file is provided:

  ```sh
  roslaunch aruco_description spawn_aruco.launch aruco_dictionary:="DICT_4X4_50" aruco_id:="1"
  ```

  In this launch file we are also loading the aruco code definition in the parameter server as `DICT_4X4_50_id1`

- load the aruco urdf inside `my_aruco_description` parameter in the ROS parameters server

  ```sh
  roslaunch aruco_description spawn_aruco.launch aruco_dictionary:="DICT_4X4_50" aruco_id:="1" aruco_description_name:="my_aruco_description"
  ```

- include the macro in my custom xacro. Here we attach the aruco marker to the `my_awesome_link` of `my_awesome_model`

  ```sh
  <link
  <?xml version='1.0' encoding='utf-8'?>
  <robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_awesome_model">
      <xacro:arg name="aruco_dictionary" default="DICT_4X4_50"/>
      <xacro:arg name="aruco_id" default="0"/>
      <xacro:arg name="aruco_name" default="DICT_4X4_50_id0"/>
      <xacro:arg name="aruco_l" default="0.2"/>
      <xacro:arg name="white_border" default="0.1"/>
      <xacro:include filename="$(find aruco_description)/urdf/general_aruco.xacro"/>

  	<link name="my_awesome_link"/>

      <xacro:generic_aruco aruco_dictionary="$(arg aruco_dictionary)" aruco_id="$(arg aruco_id)" aruco_l="$(arg aruco_l)" aruco_name="$(arg aruco_name)" white_border="$(arg white_border)" connected_to="my_awesome_link">
          <origin xyz="0 0 0" rpy="0 0 0" />
      </xacro:generic_aruco>

  </robot>
  ```

  It is possible to specify a custom size of the aruco code, a custom id and dictionary. Also the dimension of the white border and the name of the model.

## Launch files

* **generate_aruco.launch:** generate the required aruco code meshes

  Arguments:

  - **`aruco_dictionary`** Aruco dictionary to use. Available  Check [here](#Supported-dictionaries) for the available list of dictionaries.

  - **`aruco_ids`** Aruco ids to generate. If `aruco_ids:=-1` all Aruco of the dictionary are generated. This can be a single value e.g. `aruco_ids:=1` or also an array of values `aruco_ids:=1,2,3,4`

* **load_aruco_description.launch** load the aruco description as a parameter in the ROS parameter server
  Arguments:

  * **`aruco_dictionary`** dictionary where to take the aruco code
  * **`aruco_id`** id of the aruco to store in the parameter
  * **`aruco_description_name`** name of the parameter that will be used to store the aruco description in the ROS parameter server

* **spawn_aruco.launch** load the aruco code description in the parameter server and spawn the code in gazebo
  Arguments:

  * **`x`** spawn position of the aruco code along x axis
  * **`y`** spawn position of the aruco code along y axis
  * **`z`** spawn position of the aruco code along z axis
  * **`roll`** spawn roll angle of the aruco code
  * **`pitch`** spawn pitch angle of the aruco code
  * **`yaw`** spawn yaw angle of the aruco code
  * **`aruco_dictionary`** dictionary where to take the aruco code
  * **`aruco_id`** id of the aruco to store in the parameter

## Supported dictionaries

These are all the strings that can be used as **`aruco_dictionary`**

- DICT_4X4_50
- DICT_4X4_100
- DICT_4X4_250
- DICT_4X4_1000
- DICT_5X5_50
- DICT_5X5_100
- DICT_5X5_250
- DICT_5X5_1000
- DICT_6X6_50
- DICT_6X6_100
- DICT_6X6_250
- DICT_6X6_1000
- DICT_7X7_50
- DICT_7X7_100
- DICT_7X7_250
- DICT_7X7_1000

## Credits
- The script to generate the png images of the Aruco code is based on Adrian Rosebrock work that can be found at this [link](https://www.pyimagesearch.com/2020/12/14/generating-aruco-markers-with-opencv-and-python/)
- The dae template and the xacro definition is extracted from [RDaneelOlivav](https://gist.github.com/RDaneelOlivav)  work and can be found [here](https://gist.github.com/RDaneelOlivav/990addc733fbeb8549c3979d5bca41b2)
- The README template is taken from [ethz-asl](https://github.com/ethz-asl/ros_best_practices.git)
