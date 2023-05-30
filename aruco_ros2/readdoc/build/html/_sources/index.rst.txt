.. Aruco_Ro2 documentation master file, created by
   sphinx-quickstart on Wed Jul 14 08:34:58 2021.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to Aruco Ros 2's documentation!
=======================================

The purpose of Aruco Augmented Reality marker detector for ROS2 is to provide the user 
to have a ROS2 wrappers on Aruco Augmented Reality marker detector library. Inside this 
ROS2 wrapper will be using Aruco library created by 2011 Rafael Muñoz Salinas and modified 
from existing ROS wrapper by Bence Magyar (link). It involves application of marker pose 
estimation and Visual servoing, which tracks multiple markers at the same time.

The Aruco Augmented Reality marker detector’s process is based on finding correspondences 
between points using of Binary Square fiducial in the real environment and their 2d image
projection. The main benefit of these markers is that a single marker provides four
corners of fiducial marker to obtain the camera pose. The inner binary codification 
in an ArUco markers is a synthetic square marker composed by a wide black border and 
an inner binary matrix which determines ID, which makes them robust, allowing the 
possibility of applying error detection and correction techniques.

.. toctree::
   :maxdepth: 2
   :caption: Contents:



Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

Table of Contents
^^^^^^^^^^^^^^^^^

.. toctree::
    :maxdepth: 2

    self
    license.rst
    installation.rst
    quick_run.rst

