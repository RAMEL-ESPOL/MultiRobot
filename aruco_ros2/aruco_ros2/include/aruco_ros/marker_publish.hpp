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
* @file simple_double.cpp
* @author Raymond Chan Chee Leong  (Bence Magyar-> ROS1)
* @date June 2020
* @version 0.1
* @brief Modified copy of simple_single.cpp ROS 2 to publish all visible markers and pose of markers.
*/


#ifndef ARUCO_ROS__MARKER_PUBLISH_HPP_
#define ARUCO_ROS__MARKER_PUBLISH_HPP_

#include <iostream>
#include <functional>
#include <vector>
#include <string>
#include <memory>
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/convert.h"
#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include "sensor_msgs/image_encodings.hpp"
#include "aruco_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"
#include "aruco_ros/aruco_ros_utils.hpp"
#include "aruco_msgs/msg/marker_array.hpp"
#include <tf2_ros/buffer.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>


/**
 * @class  ArucoMarkerPublisher
 * @brief   all marker detector with pose
 *
 */
class ArucoMarkerPublisher : public rclcpp::Node
{
public:
  // aruco stuff
  aruco::MarkerDetector mDetector_;
  aruco::CameraParameters camParam_;
  vector<aruco::Marker> markers_;

  // node params
  bool useRectifiedImages_;
  std::string marker_frame_;
  std::string camera_frame_;
  std::string reference_frame_;
  double marker_size_;
  bool rotate_marker_axis_;
  bool cam_info_received;

  // ROS pub-sub

  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub;

  image_transport::Publisher image_pub_;
  image_transport::Publisher debug_pub_;
  rclcpp::Publisher<aruco_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr marker_list_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::shared_ptr<tf2_ros::TransformListener> tfListener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub_;
  aruco_msgs::msg::MarkerArray::Ptr marker_msg_;
  cv::Mat inImage_;
  bool useCamInfo_;
  std_msgs::msg::UInt32MultiArray marker_list_msg_;

public:
/**
 * @brief ArucoMarkerPublisher constructor for ArucoMarkerPublisher
 * @param option   input rclcpp node option
 */
  explicit ArucoMarkerPublisher(rclcpp::NodeOptions options);
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
 * @brief arucoMarker2Tf convert the marker pose info to TF2 transform type
 * @param marker   input marker
 * @param rotate_marker_axis if true, Rotate axis direction as to fit from opencv to ROS frame.
 * @return transformation of marker in type TF2
 */

public:
  void cam_info_callback(sensor_msgs::msg::CameraInfo::ConstPtr msg);
};
#endif  // ARUCO_ROS__MARKER_PUBLISH_HPP_
