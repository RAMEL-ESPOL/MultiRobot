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

cv::Vec3d aruco_ros::rotationVectorWithROSAxes(cv::Vec3d &Rvec) {
  cv::Mat rot(3, 3, CV_64FC1);
  cv::Rodrigues(Rvec, rot);

  std::cout << "Rvec_USED: " << " ";
  std::cout << Rvec << std::endl;

  std::cout << "rot: " << " ";
  std::cout << rot << std::endl;

  // Rotate axis direction as to fit ROS
  cv::Mat rotate_to_ros =
      (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, -1, 0, 0, 0, -1);

  std::cout << "rotate_to_ros: " << " ";
  std::cout << rotate_to_ros << std::endl;

  cv::Mat rot2 = rot * rotate_to_ros;

  std::cout << "rot new: " << " ";
  std::cout << rot2 << std::endl;


  cv::Vec3d ret;
  cv::Rodrigues(rot2, ret);

  std::cout << "ret: " << " ";
  std::cout << ret << std::endl;

  return ret;
}

cv::Vec3d aruco_ros::rotationVectorwrtCamera(cv::Vec3d &Rvec) {
  cv::Mat rot(3, 3, CV_64FC1);
  cv::Rodrigues(Rvec, rot);

  // Rotate axis direction as to fit Camera img
  cv::Mat rotate_to_cmr =
      (cv::Mat_<double>(3, 3) << 0, 0, 1, 0, 1, 0, -1, 0, 0);
      //(cv::Mat_<double>(3, 3) << 0, 0, 1, 0, -1, 0, -1, 0, 0);
      //(cv::Mat_<double>(3, 3) << 0.6135935, 0.7728130, -0.1620590, 0.7728130, -0.5456259, 0.3241180, 0.1620590, -0.3241180, -0.9320324);
      //(cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);

  cv::Mat rot2 = rot * rotate_to_cmr;

  cv::Vec3d ret;
  cv::Rodrigues(rot2, ret);

  return ret;
}

void aruco_ros::draw_axis(const cv::Mat  &image, const cv::InputArray &cameraMatrix, const cv::InputArray &distCoeffs,
                   const cv::InputArray &rvec, const cv::InputArray &tvec, float length, float markerLength, int thickness) 
{
  /*cv::Mat objPoints(4, 1, CV_32FC3);
  objPoints.ptr<cv::Vec3f>(0)[0] = cv::Vec3f(-markerLength/2.f, markerLength/2.f, 0);
  objPoints.ptr<cv::Vec3f>(0)[1] = cv::Vec3f(markerLength/2.f, markerLength/2.f, 0);
  objPoints.ptr<cv::Vec3f>(0)[2] = cv::Vec3f(markerLength/2.f, -markerLength/2.f, 0);
  objPoints.ptr<cv::Vec3f>(0)[3] = cv::Vec3f(-markerLength/2.f, -markerLength/2.f, 0);
*/

  float size= markerLength;
  cv::Mat objectPoints (4,3,CV_32FC1);
  objectPoints.at<float>(0,0)=0;
  objectPoints.at<float>(0,1)=0;
  objectPoints.at<float>(0,2)=0;
  objectPoints.at<float>(1,0)=size;
  objectPoints.at<float>(1,1)=0;
  objectPoints.at<float>(1,2)=0;
  objectPoints.at<float>(2,0)=0;
  objectPoints.at<float>(2,1)=size;
  objectPoints.at<float>(2,2)=0;
  objectPoints.at<float>(3,0)=0;
  objectPoints.at<float>(3,1)=0;
  objectPoints.at<float>(3,2)=size;

  vector<cv::Point2f> imagePoints;
  projectPoints( objectPoints, rvec, tvec, cameraMatrix , distCoeffs,   imagePoints);

  //draw lines of different colours
  cv::line(image,imagePoints[0],imagePoints[1],cv::Scalar(255,0,0,255),thickness,cv::LINE_AA);
  cv::line(image,imagePoints[0],imagePoints[2],cv::Scalar(0,255,0,255),thickness,cv::LINE_AA);
  cv::line(image,imagePoints[0],imagePoints[3],cv::Scalar(0,0,255,255),thickness,cv::LINE_AA);
  //putText(image,"x", imagePoints[1],FONT_HERSHEY_SIMPLEX, 0.6, Scalar(255,0,0,255),2);
  //putText(image,"y", imagePoints[2],FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,255,0,255),2);
  //putText(image,"z", imagePoints[3],FONT_HERSHEY_SIMPLEX, 0.6, Scalar(0,0,255,255),2);

}