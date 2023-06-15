/*****************************
Detect and Draw Aruco Markers.
********************************/

/**
* @file marker_publish.cpp
* @author Ramel
* @date June 2023
* @version 1.0
* @brief Detect and Draw Aruco Markers, and broadcast TFs to Rviz
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
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/convert.h"
#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "aruco_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"
#include "aruco_ros/aruco_ros_utils.hpp"
#include <aruco/marker.h>
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

  // camera params
  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;

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
  cv::Mat inImage_, inImage_copy;
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
