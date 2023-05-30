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

#include <string>
#include <memory>
#include "aruco_ros/marker_publish.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_interfaces/node_topics_interface.hpp>

ArucoMarkerPublisher::ArucoMarkerPublisher(rclcpp::NodeOptions options)
: cam_info_received(false),
  useCamInfo_(true),
  Node("aruco_marker_publisher", options),
  tf_buffer_(std::make_shared<tf2_ros::Buffer>(get_clock()))
{
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(),
    get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tfListener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster= std::make_shared<tf2_ros::TransformBroadcaster>(this);
  image_sub_ =
    image_transport::create_subscription(
    this, "/image",
    std::bind(&ArucoMarkerPublisher::image_callback, this, std::placeholders::_1), "raw");

  get_parameter_or("use_camera_info", useCamInfo_, true);

  cam_info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info",
    rclcpp::SensorDataQoS(),
    std::bind(&ArucoMarkerPublisher::cam_info_callback, this, std::placeholders::_1));
  if (useCamInfo_) {
    get_parameter_or("marker_size", marker_size_, 0.1);
    get_parameter_or("image_is_rectified", useRectifiedImages_, false);
    get_parameter_or("reference_frame", reference_frame_, string{""});
    get_parameter_or("camera_frame", camera_frame_, string{"camera_color_frame"});
    get_parameter_or("rotate_marker_axis", rotate_marker_axis_, false);
    if (reference_frame_.empty()) {
      reference_frame_ = camera_frame_;
    }
  } else {
    camParam_ = aruco::CameraParameters();
  }
  image_pub_ = image_transport::create_publisher(this, "result");
  debug_pub_ = image_transport::create_publisher(this, "debug");
  marker_pub_ = create_publisher<aruco_msgs::msg::MarkerArray>("markers", 100);
  marker_list_pub_ = create_publisher<std_msgs::msg::UInt32MultiArray>("markers_list", 10);
  marker_msg_ = aruco_msgs::msg::MarkerArray::Ptr(new aruco_msgs::msg::MarkerArray());
  marker_msg_->header.frame_id = reference_frame_;
}

bool ArucoMarkerPublisher::getTransform(
  const std::string & refFrame,
  const std::string & childFrame,
  geometry_msgs::msg::TransformStamped & transform)
{
  std::string errMsg;
  if (!tf_buffer_->canTransform(
      refFrame,
      childFrame,
      tf2::TimePointZero,
      tf2::durationFromSec(1.0),
      &errMsg))
  {
    RCLCPP_ERROR_STREAM(get_logger(), "Unable to get pose from TF: " << errMsg);
    return false;
  } else {
    try {
      transform = tf_buffer_->lookupTransform(
        refFrame, childFrame,
        tf2::TimePointZero);
    } catch (const tf2::TransformException & e) {
      RCLCPP_ERROR_STREAM(
        get_logger(), "Error in lookupTransform of " << childFrame << " in " << refFrame);
      return false;
    }
  }
  return true;
}

void ArucoMarkerPublisher::image_callback(const sensor_msgs::msg::Image::ConstPtr & msg)
{
  std::cout << "check call back" << std::endl;

  if (!useCamInfo_ || cam_info_received) {
    bool publishImage = image_pub_.getNumSubscribers() > 0;
    bool publishDebug = debug_pub_.getNumSubscribers() > 0;

    rclcpp::Time curr_stamp(now());
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      inImage_ = cv_ptr->image;

      // clear out previous detection results
      markers_.clear();

      // Ok, let's detect
      mDetector_.detect(inImage_, markers_, camParam_, marker_size_, false);

      // marker array publish

      {
        marker_msg_->markers.clear();
        marker_msg_->markers.resize(markers_.size());
        marker_msg_->header.stamp = curr_stamp;

        for (size_t i = 0; i < markers_.size(); ++i) {
          aruco_msgs::msg::Marker & marker_i = marker_msg_->markers.at(i);
          marker_i.header.stamp = curr_stamp;
          marker_i.id = markers_.at(i).id;
          marker_i.confidence = 1.0;
        }

        // if there is camera info let's do 3D stuff
        if (useCamInfo_) {
          geometry_msgs::msg::TransformStamped cameraToReference;
          tf2::Transform cameraToReference_tf;

          if (reference_frame_ != camera_frame_) {
            if (!getTransform(reference_frame_, camera_frame_, cameraToReference)) {
              return;
            }
            tf2::fromMsg(cameraToReference.transform, cameraToReference_tf);
          } else {
            cameraToReference_tf.setIdentity();
          }

          // Now find the transform for each detected marker
          for (size_t i = 0; i < markers_.size(); ++i) {
            aruco_msgs::msg::Marker & marker_i = marker_msg_->markers.at(i);
            tf2::Transform transform = aruco_ros::arucoMarker2Tf(markers_[i], rotate_marker_axis_);

            std::cout << transform.getOrigin().getX() << " || " << transform.getOrigin().getY() <<
              " || " << transform.getOrigin().getZ() << endl;
            transform = cameraToReference_tf * transform;
            geometry_msgs::msg::TransformStamped tf_msg;
	    tf_msg.header.stamp = curr_stamp;
	    tf_msg.header.frame_id = camera_frame_;
	    tf_msg.child_frame_id = "marker_frame_" + std::to_string(markers_.at(i).id);
	    tf_msg.transform.translation.x = transform.getOrigin().getX();
	    tf_msg.transform.translation.y = transform.getOrigin().getY();
	    tf_msg.transform.translation.z = transform.getOrigin().getZ();
	    tf_msg.transform.rotation.x = transform.getRotation().getX();
	    tf_msg.transform.rotation.y = transform.getRotation().getY();
	    tf_msg.transform.rotation.z = transform.getRotation().getZ();
	    tf_msg.transform.rotation.w = transform.getRotation().getW();
	    tf_broadcaster->sendTransform(tf_msg);
	    tf2::toMsg(transform, marker_i.pose.pose);
            marker_i.header.frame_id = reference_frame_;
          }
        }

        // publish marker array
        if (marker_msg_->markers.size() > 0) {
          marker_pub_->publish(*marker_msg_);
        }
      }

      {
        marker_list_msg_.data.resize(markers_.size());
        for (size_t i = 0; i < markers_.size(); ++i) {
          marker_list_msg_.data[i] = markers_[i].id;
        }

        marker_list_pub_->publish(marker_list_msg_);
      }

      // Draw detected markers on the image for visualization
      /*
          for(size_t i=0; i<markers_.size(); ++i)
          {
              markers_[i].draw(inImage_,cv::Scalar(0,0,255),2);

              if(markers_[i].id == 101){
                   putText(inImage_,"      Bed = 1",markers_[i].getCenter() + cv::Point2f(0,0),cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0,0,255,255),2);
              }
              if(markers_[i].id == 201){
                   putText(inImage_,"      Bed = 2", markers_[i].getCenter(),cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0,0,255,255),2);
              }
              if(markers_[i].id == 301){
                   putText(inImage_,"      Bed = 3", markers_[i].getCenter(),cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0,0,255,255),2);
              }
              if(markers_[i].id == 401){
                   putText(inImage_,"      Bed = 4", markers_[i].getCenter(),cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0,0,255,255),2);
              }
              if(markers_[i].id == 701){
                   putText(inImage_,"        Docking station", markers_[i].getCenter(),cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0,0,255,255),2);
              }

          }*/

      // draw a 3d cube in each marker if there is 3d info
      if (camParam_.isValid() && marker_size_ > 0) {
        for (size_t i = 0; i < markers_.size(); ++i) {
          aruco::CvDrawingUtils::draw3dAxis(inImage_, markers_[i], camParam_);
        }
      }

      // cv::namedWindow("Camera view",cv::WINDOW_NORMAL);
      // cv::resizeWindow("Camera view", 800, 450);
      // cv::imshow("Camera view",inImage_);
      // cv::waitKey(1);

      // publish input image with markers drawn on it
      if (publishImage) {
        // show input with augmented information
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.header.frame_id = camera_frame_;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage_;
        image_pub_.publish(out_msg.toImageMsg());
      }

      // publish image after internal image processing
      if (publishDebug) {
        // show also the internal image resulting from the threshold operation
        cv_bridge::CvImage debug_msg;
        debug_msg.header.stamp = curr_stamp;
        debug_msg.header.frame_id = camera_frame_;
        debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_msg.image = mDetector_.getThresholdedImage();
        debug_pub_.publish(debug_msg.toImageMsg());
      }
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
    }
  }
}

void ArucoMarkerPublisher::cam_info_callback(sensor_msgs::msg::CameraInfo::ConstPtr msg)
{
  if (useCamInfo_) {
    camParam_ = aruco_ros::rosCameraInfo2ArucoCamParams(*msg, useRectifiedImages_);

    cam_info_received = true;
  }
}
