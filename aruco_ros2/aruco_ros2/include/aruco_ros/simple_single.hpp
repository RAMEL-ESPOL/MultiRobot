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


/**
* @file simple_single.cpp
* @author Raymond Chan Chee Leong  (Bence Magyar-> ROS1)
* @date June 2020
* @version 0.1
* @brief ROS2 version of the example named "simple_single"  in the Aruco software package. this is to show 1 specific ID of aruco detected pose
*/
#ifndef ARUCO_ROS__SIMPLE_SINGLE_HPP_
#define ARUCO_ROS__SIMPLE_SINGLE_HPP_

#include <functional>
#include <iostream>
#include <vector>
#include <string>
#include <memory>
#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "aruco_ros/aruco_ros_utils.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <tf2_ros/buffer.h>

/**
 * @class  ArucoSimple
 * @brief   simple aruco for single detection
 *
 */
class ArucoSimple : public rclcpp::Node
{
public:
  cv::Mat inImage;
  aruco::CameraParameters camParam;
  tf2::Transform rightToLeft;
  bool useRectifiedImages;
  aruco::MarkerDetector mDetector;
  std::vector<aruco::Marker> markers;
  std::string refinementMethod;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub;
  bool cam_info_received;
  image_transport::Publisher image_pub;
  image_transport::Publisher debug_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr transform_pub;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr position_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pixel_pub;
  std::string marker_frame;
  std::string camera_frame;
  std::string reference_frame;
  std::shared_ptr<tf2_ros::TransformListener> _tfListener;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  double marker_size;
  int marker_id;
  bool rotate_marker_axis_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> br;
  image_transport::Subscriber image_sub;

public:
/**
 * @brief ArucoSimple constructor for ArucoSimple
 * @param option   input rclcpp node option
 */
  explicit ArucoSimple(rclcpp::NodeOptions options);
  /**
 * @brief getTransform get transformation from 2 frame
 * @param refFrame   input reference frame
 * @param childFrame   input child frame
 * @param transform  output transformation
 * @return true if success, false if fail to get transform
 */
  bool getTransform(
    const std::string & refFrame,
    const std::string & childFrame,
    geometry_msgs::msg::TransformStamped & transform);
/**
 * @brief image topic subscribe callback
 * @param msg   image message pointer
 */
  void image_callback(const sensor_msgs::msg::Image::ConstPtr & msg);
/**
 * @brief camera info topic subscribe callback
 * @param msg   camera info message pointer
 */
  // wait for one camerainfo, then shut down that subscriber
  void cam_info_callback(sensor_msgs::msg::CameraInfo::ConstPtr msg);
};
#endif  // ARUCO_ROS__SIMPLE_SINGLE_HPP_
