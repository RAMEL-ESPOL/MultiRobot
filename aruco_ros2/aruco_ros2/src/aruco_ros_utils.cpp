// Copyright 2011 Rafael Muñoz Salinas
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/*****************************
The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
********************************/


#include <iostream>
#include "aruco_ros/aruco_ros_utils.hpp"
#include "rclcpp/logging.hpp"

// #include "tf2/transform_datatypes.h"
// #include "tf2/LinearMath/Transform.h"

cv::Vec3d aruco_ros::rotationVectorWithROSAxes(const cv::Vec3d &Rvec) {
  cv::Mat rot(3, 3, CV_64FC1);
  cv::Rodrigues(Rvec, rot);

  // Rotate axis direction as to fit ROS
  cv::Mat rotate_to_ros =
      //(cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
      (cv::Mat_<double>(3, 3) << 0, 1, 0, 0, 0, -1, -1, 0, 0);
      //(cv::Mat_<double>(3, 3) << 0, -1, 0, 1, 0, 0, 0, 0, 1);
      //(cv::Mat_<double>(3, 3) << 0, 0, 1, 0, -1, 0, 1, 0, 0);
      //(cv::Mat_<double>(3, 3) << -1, 0, 0, 0, 0, 1, 0, 1, 0);
  rot = rot * rotate_to_ros.t();

  cv::Vec3d ret;
  cv::Rodrigues(rot, ret);
  return ret;
}

aruco::CameraParameters aruco_ros::rosCameraInfo2ArucoCamParams(
  const sensor_msgs::msg::CameraInfo & cam_info,
  bool useRectifiedParameters)
{
  cv::Mat cameraMatrix(3, 3, CV_64FC1);
  cv::Mat distorsionCoeff(4, 1, CV_64FC1);
  cv::Size size(cam_info.width, cam_info.height);

  if (useRectifiedParameters) {
    cameraMatrix.setTo(0);
    cameraMatrix.at<double>(0, 0) = cam_info.p[0];   cameraMatrix.at<double>(0, 1) = cam_info.p[1];
    cameraMatrix.at<double>(0, 2) = cam_info.p[2];
    cameraMatrix.at<double>(1, 0) = cam_info.p[4];   cameraMatrix.at<double>(1, 1) = cam_info.p[5];
    cameraMatrix.at<double>(1, 2) = cam_info.p[6];
    cameraMatrix.at<double>(2, 0) = cam_info.p[8];   cameraMatrix.at<double>(2, 1) = cam_info.p[9];
    cameraMatrix.at<double>(2, 2) = cam_info.p[10];

    for (int i = 0; i < 4; ++i) {
      distorsionCoeff.at<double>(i, 0) = 0;
    }
  } else {
    for (int i = 0; i < 9; ++i) {
      cameraMatrix.at<double>(i % 3, i - (i % 3) * 3) = cam_info.k[i];
    }

    if (cam_info.d.size() == 4) {
      for (int i = 0; i < 4; ++i) {
        distorsionCoeff.at<double>(i, 0) = cam_info.d[i];
      }
    } else {
      RCLCPP_WARN(
        rclcpp::get_logger(
          "LifecyclePublisher"),
        "length of camera_info D vector is not 4, assuming zero distortion...");
      for (int i = 0; i < 4; ++i) {
        distorsionCoeff.at<double>(i, 0) = 0;
      }
    }
  }

  return aruco::CameraParameters(cameraMatrix, distorsionCoeff, size);
}

tf2::Transform aruco_ros::arucoMarker2Tf(const aruco::Marker & marker, bool rotate_marker_axis)
{
  aruco::Marker dmarker;
  aruco::MarkerDetector mdetec;
  cv::Mat rot(3, 3, CV_64FC1);
  cv::Mat Rvec64;
  marker.Rvec.convertTo(Rvec64, CV_64FC1);
  cv::Rodrigues(Rvec64, rot);
  cv::Mat tran64;
  marker.Tvec.convertTo(tran64, CV_64FC1);

  // Rotate axis direction as to fit from opencv to ROS frame
  if (rotate_marker_axis) {
    cv::Mat rotate_to_ros(3, 3, CV_64FC1);
    // -1 0 0
    // 0 0 1
    // 0 1 0
    rotate_to_ros.at<double>(0, 0) = -1.0;
    rotate_to_ros.at<double>(0, 1) = 0.0;
    rotate_to_ros.at<double>(0, 2) = 0.0;
    rotate_to_ros.at<double>(1, 0) = 0.0;
    rotate_to_ros.at<double>(1, 1) = 0.0;
    rotate_to_ros.at<double>(1, 2) = 1.0;
    rotate_to_ros.at<double>(2, 0) = 0.0;
    rotate_to_ros.at<double>(2, 1) = 1.0;
    rotate_to_ros.at<double>(2, 2) = 0.0;
    rot = rot * rotate_to_ros.t();
  }
  tf2::Matrix3x3 tf_rot(rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
    rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
    rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2));


  tf2::Vector3 tf_orig(tran64.at<double>(2, 0), -tran64.at<double>(0, 0), -tran64.at<double>(1, 0));


  return tf2::Transform(tf_rot, tf_orig);
}
