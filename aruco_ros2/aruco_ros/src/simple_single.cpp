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

#include <memory>
#include <string>
#include "aruco_ros/simple_single.hpp"
#include <tf2_ros/buffer.h>

/**
 * @class  ArucoSimple
 * @brief   simple aruco for single detection
 *
 */

ArucoSimple::ArucoSimple(rclcpp::NodeOptions options)
: cam_info_received(false),
  Node("aruco_node", options),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock()))
{
  /*
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  _tfListener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


  // Print parameters of aruco marker detector:
  RCLCPP_INFO_STREAM(
    get_logger(), "Corner refinement method: " << mDetector.getCornerRefinementMethod());
  RCLCPP_INFO_STREAM(get_logger(), "Threshold method: " << mDetector.getThresholdMethod());
  double th1, th2;
  mDetector.getThresholdParams(th1, th2);
  RCLCPP_INFO_STREAM(
    get_logger(), "Threshold method: " <<
      " th1: " << th1 << " th2: " << th2);
  float mins, maxs;
  mDetector.getMinMaxSize(mins, maxs);
  RCLCPP_INFO_STREAM(get_logger(), "Marker size min: " << mins << "  max: " << maxs);
  RCLCPP_INFO_STREAM(get_logger(), "Desired speed: " << mDetector.getDesiredSpeed());
  image_sub =
    image_transport::create_subscription(
    this, "/image",
    std::bind(
      &ArucoSimple::image_callback,
      this, std::placeholders::_1), "raw");
  cam_info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info",
    rclcpp::SensorDataQoS(),
    std::bind(&ArucoSimple::cam_info_callback, this, std::placeholders::_1));
  image_pub = image_transport::create_publisher(this, "result");
  debug_pub = image_transport::create_publisher(this, "debug");
  pose_pub = create_publisher<geometry_msgs::msg::PoseStamped>(
    "pose/aruco2",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  transform_pub = create_publisher<geometry_msgs::msg::TransformStamped>(
    "transform",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  position_pub = create_publisher<geometry_msgs::msg::Vector3Stamped>(
    "position",
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  marker_pub = create_publisher<visualization_msgs::msg::Marker>(
    "marker", 1);
  pixel_pub = create_publisher<geometry_msgs::msg::PointStamped>(
    "pixel",
    rclcpp::SensorDataQoS());
  get_parameter_or("marker_size", marker_size, 0.05);
  get_parameter_or("marker_id", marker_id, 582);
  get_parameter_or("reference_frame", reference_frame, string(""));
  get_parameter_or("camera_frame", camera_frame, string("camera_frame"));
  get_parameter_or("marker_frame", marker_frame, string("marker_frame"));
  get_parameter_or("image_is_rectified", useRectifiedImages, false);
  get_parameter_or("rotate_marker_axis", rotate_marker_axis_, true);
  get_parameter_or("refinementMethod", refinementMethod, string(""));

  if (refinementMethod == "SUBPIX") {
    mDetector.setCornerRefinementMethod(aruco::MarkerDetector::SUBPIX);
  } else if (refinementMethod == "HARRIS") {
    mDetector.setCornerRefinementMethod(aruco::MarkerDetector::HARRIS);
  } else if (refinementMethod == "NONE") {
    mDetector.setCornerRefinementMethod(aruco::MarkerDetector::NONE);
  } else {
    mDetector.setCornerRefinementMethod(aruco::MarkerDetector::LINES);
  }
  if (reference_frame.empty()) {
    reference_frame = camera_frame;
  }
  RCLCPP_INFO(
    get_logger(), "Aruco node started with marker size of %f m and marker id to track: %d",
    marker_size, marker_id);
  RCLCPP_INFO(
    get_logger(), "Aruco node will publish pose to TF with %s as parent and %s as child.",
    reference_frame.c_str(), marker_frame.c_str());
  mDetector.setThresholdParams(7, 7);
  br = std::make_shared<tf2_ros::TransformBroadcaster>(this);
}

bool ArucoSimple::getTransform(
  const std::string & refFrame,
  const std::string & childFrame,
  geometry_msgs::msg::TransformStamped & transform)
{
  std::string errMsg;
  if (!tf_buffer_->canTransform(
      refFrame,
      childFrame,
      rclcpp::Time(0),
      rclcpp::Duration(0, 500000000),
      &errMsg))
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Unable to get pose from TF: " << errMsg << " YY");
    return false;
  } else {
    try {
      transform = tf_buffer_->lookupTransform(
        refFrame, childFrame,
        rclcpp::Time(0),
        rclcpp::Duration(0, 500000000));
    } catch (const tf2::TransformException & e) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Error in lookupTransform of " << childFrame << " in " << refFrame);
      return false;
    }
  }
  return true;
}

void ArucoSimple::image_callback(const sensor_msgs::msg::Image::ConstPtr & msg)
{
  if (cam_info_received) {
    rclcpp::Time curr_stamp(now());
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      inImage = cv_ptr->image;
      // detection results will go into "markers"
      markers.clear();
      // Ok, let's detect
      mDetector.detect(inImage, markers, camParam, marker_size, false);
      // for each marker, draw info and its boundaries in the image
      for (size_t i = 0; i < markers.size(); ++i) {
        // only publishing the selected marker
        if (markers[i].id == marker_id) {
          tf2::Transform transform = aruco_ros::arucoMarker2Tf(markers[i], rotate_marker_axis_);
          geometry_msgs::msg::TransformStamped cameraToReference;
          tf2::Transform cameraToReference_tf = tf2::Transform::getIdentity();
          // <false,true>=> tf to msg, <true,false>=> msg to tf
          tf2::impl::Converter<false, true>::convert(
            cameraToReference_tf,
            cameraToReference.transform);
          if (reference_frame != camera_frame) {
            if (!getTransform(reference_frame, camera_frame, cameraToReference)) {
              return;
            }
          }
          tf2::impl::Converter<true, false>::convert(
            cameraToReference.transform,
            cameraToReference_tf);
          transform =
            static_cast<tf2::Transform>(cameraToReference_tf) *
            static_cast<tf2::Transform>(rightToLeft) * transform;
          geometry_msgs::msg::TransformStamped stampedTransform;
          stampedTransform.header.frame_id = reference_frame;
          stampedTransform.header.stamp = curr_stamp;
          stampedTransform.child_frame_id = marker_frame;
          tf2::impl::Converter<false, true>::convert(transform, stampedTransform.transform);
          br->sendTransform(stampedTransform);
          geometry_msgs::msg::PoseStamped poseMsg;
          tf2::toMsg(transform, poseMsg.pose);
          poseMsg.header.frame_id = reference_frame;
          poseMsg.header.stamp = curr_stamp;
          pose_pub->publish(poseMsg);
          transform_pub->publish(stampedTransform);
          geometry_msgs::msg::Vector3Stamped positionMsg;
          positionMsg.header = stampedTransform.header;
          positionMsg.vector = stampedTransform.transform.translation;
          position_pub->publish(positionMsg);
          geometry_msgs::msg::PointStamped pixelMsg;
          pixelMsg.header = stampedTransform.header;
          pixelMsg.point.x = markers[i].getCenter().x;
          pixelMsg.point.y = markers[i].getCenter().y;
          pixelMsg.point.z = 0;
          pixel_pub->publish(pixelMsg);
          // Publish rviz marker representing the ArUco marker patch
          visualization_msgs::msg::Marker visMarker;
          visMarker.header = stampedTransform.header;
          visMarker.id = 1;
          visMarker.type = visualization_msgs::msg::Marker::CUBE;
          visMarker.action = visualization_msgs::msg::Marker::ADD;
          visMarker.pose = poseMsg.pose;
          visMarker.scale.x = marker_size;
          visMarker.scale.y = 0.001;
          visMarker.scale.z = marker_size;
          visMarker.color.r = 1.0;
          visMarker.color.g = 0;
          visMarker.color.b = 0;
          visMarker.color.a = 1.0;
          visMarker.lifetime = rclcpp::Duration(3, 0);
          marker_pub->publish(visMarker);
        }
        // but drawing all the detected markers
        markers[i].draw(inImage, cv::Scalar(0, 0, 255), 2);
      }
      // draw a 3d cube in each marker if there is 3d info
      if (camParam.isValid() && marker_size > 0) {
        for (size_t i = 0; i < markers.size(); ++i) {
          aruco::CvDrawingUtils::draw3dAxis(inImage, markers[i], camParam);
        }
      }
      if (image_pub.getNumSubscribers() > 0) {
        // show input with augmented information
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage;
        image_pub.publish(out_msg.toImageMsg());
      }
      // cv::imshow("display check", inImage);
      // cv::waitKey(1);
      if (debug_pub.getNumSubscribers() > 0) {
        // show also the internal image resulting from the threshold operation
        cv_bridge::CvImage debug_msg;
        debug_msg.header.stamp = curr_stamp;
        debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_msg.image = mDetector.getThresholdedImage();
        debug_pub.publish(debug_msg.toImageMsg());
      }
    } catch (cv_bridge::Exception & e) {
      std::cout << "try failed" << std::endl;
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }
  }
}

void ArucoSimple::cam_info_callback(sensor_msgs::msg::CameraInfo::ConstPtr msg)
{
  camParam = aruco_ros::rosCameraInfo2ArucoCamParams(*msg, useRectifiedImages);
  // handle cartesian offset between stereo pairs
  // see the sensor_msgs/CamaraInfo documentation for details
  rightToLeft.setIdentity();
  rightToLeft.setOrigin(
    tf2::Vector3(
      -msg->p[3] / msg->p[0],
      -msg->p[7] / msg->p[5],
      0.0));
  cam_info_received = true;*/
}
