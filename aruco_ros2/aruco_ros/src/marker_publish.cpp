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

#include <iostream>
#include <functional>
#include <vector>
#include <string>
#include <memory>
#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include "aruco_ros/aruco_ros_utils.hpp"
#include "aruco_msgs/msg/marker_array.hpp"

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/wait_for_message.hpp"
#include <rclcpp/node_interfaces/node_topics_interface.hpp>
#include "rcpputils/asserts.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "std_msgs/msg/u_int32_multi_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

using namespace std::chrono_literals;

class ArucoMarkerPublisher : public rclcpp::Node
{
private:
  rclcpp::Node::SharedPtr subNode;
  // ArUco stuff
  aruco::CameraParameters camParam_;
  std::vector<aruco::Marker> markers_;

  // node params
  bool useRectifiedImages_;
  std::string camera_frame_;
  std::string reference_frame_;
  std::string topic_;
  double marker_size_;
  
  // camera params
  cv::Mat cameraMatrix;
  cv::Mat distCoeffs;

  // ROS pub-sub
  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr cam_info_sub;

  image_transport::Publisher image_pub_;
  image_transport::Publisher debug_pub_;
  rclcpp::Publisher<aruco_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt32MultiArray>::SharedPtr marker_list_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::shared_ptr<aruco_msgs::msg::MarkerArray> marker_msg_;
  cv::Mat inImage_, inImage_copy;
  bool useCamInfo_;
  std_msgs::msg::UInt32MultiArray marker_list_msg_;

public:
  ArucoMarkerPublisher(rclcpp::NodeOptions options)
  : Node("marker_publisher", options), useCamInfo_(true)
  //{
  //}

  //bool setup()
  {
    //subNode = this->create_sub_node(this->get_name());
    // Declare node parameters
    this->declare_parameter<double>("marker_size", 0.05);
    this->declare_parameter<std::string>("topic", "camera");
    this->declare_parameter<std::string>("reference_frame", "map");
    this->declare_parameter<std::string>("camera_frame", "camera_color_optical_frame");
    //this->declare_parameter<std::string>("aruco_dictionary_id", "DICT_4X4_250"); //To Do
    this->declare_parameter<bool>("image_is_rectified", false);
    this->declare_parameter<bool>("use_camera_info", true);
    //std::string dictionary_id_name = get_parameter("aruco_dictionary_id").as_string(); //To Do


    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster= std::make_shared<tf2_ros::TransformBroadcaster>(this);

    //it_ = std::make_unique<image_transport::ImageTransport>(shared_from_this());
    //image_sub_ = it_->subscribe("/image", 1, &ArucoMarkerPublisher::image_callback, this);
    cam_info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
                  "/camera_info",
                  rclcpp::SensorDataQoS(),
                  std::bind(&ArucoMarkerPublisher::cam_info_callback, this, std::placeholders::_1));
    image_sub_ = image_transport::create_subscription(
                  this, "/image",
                  std::bind(&ArucoMarkerPublisher::image_callback, this, std::placeholders::_1), "raw");

    this->get_parameter_or<bool>("use_camera_info", useCamInfo_, true);
    if (useCamInfo_) {
      
      //RCLCPP_INFO(this->get_logger(), "Waiting for the camera info...");
      //sensor_msgs::msg::CameraInfo camera_info;
      //rclcpp::wait_for_message<sensor_msgs::msg::CameraInfo>(
      //  camera_info,
      //  shared_from_this(), "/camera_info");
      //RCLCPP_INFO(this->get_logger(), "Successfully obtained the camera info!");
      
      this->get_parameter_or<double>("marker_size", marker_size_, 0.05);
      this->get_parameter_or<bool>("image_is_rectified", useRectifiedImages_, false);
      this->get_parameter_or<std::string>("topic", topic_, "camera");
      //this->get_parameter_or<std::string>("aruco_dictionary_id", topic_, "DICT_4X4_250"); //To Do
      this->get_parameter_or<std::string>("reference_frame", reference_frame_, "");
      this->get_parameter_or<std::string>("camera_frame", camera_frame_, "");
      //camParam_ = aruco_ros::rosCameraInfo2ArucoCamParams(camera_info, useRectifiedImages_);
      rcpputils::assert_true(
        !(camera_frame_.empty() && !reference_frame_.empty()),
        "Either the camera frame is empty and also reference frame is empty..");
      if (reference_frame_.empty()) {
        reference_frame_ = camera_frame_;
      }
    } else {
      camParam_ = aruco::CameraParameters();
    }

      // Set up publishers
    image_pub_ = image_transport::create_publisher(this, "result");
    debug_pub_ = image_transport::create_publisher(this, "debug");
    marker_pub_ = create_publisher<aruco_msgs::msg::MarkerArray>("markers", 100);
    marker_list_pub_ = create_publisher<std_msgs::msg::UInt32MultiArray>("markers_list", 10);
    marker_msg_ = aruco_msgs::msg::MarkerArray::Ptr(new aruco_msgs::msg::MarkerArray());
    marker_msg_->header.frame_id = reference_frame_;


    //image_pub_ = it_->advertise(std::string("result"), 1);
    //debug_pub_ = it_->advertise(std::string("debug"), 1);
    //marker_pub_ = subNode->create_publisher<aruco_msgs::msg::MarkerArray>("markers", 100);
    //marker_list_pub_ =
    //  subNode->create_publisher<std_msgs::msg::UInt32MultiArray>("markers_list", 10);

    //marker_msg_ = aruco_msgs::msg::MarkerArray::Ptr(new aruco_msgs::msg::MarkerArray());
    //marker_msg_->header.frame_id = reference_frame_;

    RCLCPP_INFO(this->get_logger(), "Successfully setup the marker publisher!");

    //return true;
  }

  bool getTransform(
    const std::string & refFrame, const std::string & childFrame,
    geometry_msgs::msg::TransformStamped & transform)
  {
    std::string errMsg;

    if (!tf_buffer_->canTransform(
        refFrame, childFrame, tf2::TimePointZero,
        tf2::durationFromSec(0.5), &errMsg))
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Unable to get pose from TF : " << errMsg.c_str());
      return false;
    } else {
      try {
        transform = tf_buffer_->lookupTransform(
          refFrame, childFrame, tf2::TimePointZero, tf2::durationFromSec(
            0.5));
      } catch (const tf2::TransformException & e) {
        RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "Error in lookupTransform of " << childFrame << " in " << refFrame << " : " << e.what());
        return false;
      }
    }
    return true;
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
  {

    bool publishImage = image_pub_.getNumSubscribers() > 0;
    bool publishDebug = debug_pub_.getNumSubscribers() > 0;
    std::cout << publishImage << std::endl;
    std::cout << publishDebug << std::endl;

    //if (!publishImage && !publishDebug) {
    //  return;
    //}

    //builtin_interfaces::msg::Time curr_stamp = msg->header.stamp;
    rclcpp::Time curr_stamp(now());
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
      inImage_ = cv_ptr->image;
      inImage_.copyTo(inImage_copy);

      // clear out previous detection results
      markers_.clear();

      // ok, let's detect
      std::vector<std::vector<cv::Point2f>> corners, rejected;
      std::vector<int> marker_ids;
      //mDetector_.detect(inImage_, markers_, camParam_, marker_size_, false);
      
      //Check the camera info
      std::cout << "Camera Matrix:" << std::endl;
      for (int i = 0; i < this->cameraMatrix.rows; i++) {
          for (int j = 0; j < this->cameraMatrix.cols; j++) {
              std::cout << this->cameraMatrix.at<float>(i, j) << " ";
          }
          std::cout << std::endl;
      }

      std::cout << "Distortion Coefficients:" << std::endl;
      for (int i = 0; i < this->distCoeffs.rows; i++) {
          for (int j = 0; j < this->distCoeffs.cols; j++) {
              std::cout << this->distCoeffs.at<float>(i, j) << " ";
          }
          std::cout << std::endl;
      }
      
      cv::Mat currentCameraMatrix = this->cameraMatrix;
      cv::Mat currentDistCoeffs = this->distCoeffs;

      std::cout << "MARKER SIZE!!!!!!!!!!!!!: " << " ";
      std::cout << marker_size_ << std::endl;

      cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
      cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
      cv::aruco::detectMarkers(inImage_, dictionary, corners, marker_ids, detectorParams, rejected);
      if  (marker_ids.size() > 0) {
        
        
        int nMarkers = marker_ids.size();
        std::vector<cv::Vec3d> rvecs, tvecs;
        std::cout << "currentCameraMatrix: " << " ";
        std::cout << currentCameraMatrix << std::endl;
        cv::aruco::drawDetectedMarkers(inImage_copy, corners, marker_ids);
        cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, this->cameraMatrix, this->distCoeffs, rvecs, tvecs);

        for (int i = 0; i < marker_ids.size(); ++i) {
          //cv::Mat markerImage;
          //cv::aruco::drawMarker(dictionary, marker_ids[i], 200, markerImage, 1);
          //cv::aruco::drawDetectedMarkers(inImage_, corners, ids);
          
          //for (int j = 0; j < nMarkers; j++) {
            //solvePnP(objPoints, corners.at(i), cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
          //cv::aruco::estimatePoseSingleMarkers(corners.at(i), marker_size_, cameraMatrix, distCoeffs, rvecs.at(i), tvecs.at(i));
          //}
          /*
          std::vector<std::vector<cv::Point> > contours2;
          std::vector<cv::Vec4i> hierarchy2;
          inImage_.copyTo ( thres2 );
          cv::findContours ( thres2 , contours2, hierarchy2,cv::RetrievalModes::RETR_TREE, cv::ContourApproximationModes::CHAIN_APPROX_NONE );
          vector<Point>  approxCurve;
          */
          markers_.push_back (aruco::Marker());
          //for ( int c=0;c<4;c++ )     markers_[i][c]=corners[i][c];
          markers_.back().id=marker_ids[i];  
          markers_.back().Rvec= rvecs[i];
          markers_.back().Tvec= tvecs[i];

          std::cout << "RVECS: " << " ";
          std::cout << rvecs[i] << std::endl;

          std::cout << "TVECS: " << " ";
          std::cout << tvecs[i] << std::endl;

          

        }

      // marker array publish
      //if (publishMarkers) {
        marker_msg_->markers.clear();
        marker_msg_->markers.resize(markers_.size());
        marker_msg_->header.stamp = curr_stamp;

        for (int i = 0; i < marker_ids.size(); ++i) {

          //cv::drawFrameAxes(inImage_copy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.05);

          aruco_msgs::msg::Marker & marker_i = marker_msg_->markers.at(i);
          marker_i.header.stamp = curr_stamp;
          marker_i.id = marker_ids[i];
          marker_i.confidence = 1.0;

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

          std::cout << "reference2camera tf: " << "x: ";
          std::cout << cameraToReference_tf.getOrigin().getX() << " y: ";
          std::cout << cameraToReference_tf.getOrigin().getY() << " z: ";
          std::cout << cameraToReference_tf.getOrigin().getZ() << " X: ";
          std::cout << cameraToReference_tf.getRotation().getX() << " Y: ";
          std::cout << cameraToReference_tf.getRotation().getY() << " Z: ";
          std::cout << cameraToReference_tf.getRotation().getZ() << " W: ";
          std::cout << cameraToReference_tf.getRotation().getW() << std::endl;
          //tf2::Transform transform = aruco_ros::arucoMarker2Tf(markers_[i], rotate_marker_axis_);

          /*std::cout << transform.getOrigin().getX() << " || " << transform.getOrigin().getY() <<
            " || " << transform.getOrigin().getZ() << endl;
          transform = cameraToReference_tf * transform;*/
          
          cv::Mat rot_mat1(3, 3, CV_64F);
          cv::Rodrigues(rvecs[i], rot_mat1);

          std::cout << "rot_mat1_OUT: " << " ";
          std::cout << rot_mat1 << std::endl;

          //RotationAndTranslationVectorsToTransform
          cv::Mat rot_mat(3, 3, CV_64F);
          cv::Vec3d ret_ros = aruco_ros::rotationVectorWithROSAxes(rvecs[i]);
          cv::Rodrigues(ret_ros, rot_mat);
          tf2::Matrix3x3 tf_rot(
              rot_mat.at<double>(0, 0), rot_mat.at<double>(0, 1), rot_mat.at<double>(0, 2),
              rot_mat.at<double>(1, 0), rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2),
              rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2));
          tf2::Vector3 tf_orig(tvecs[i][0], tvecs[i][1], tvecs[i][2]);
          tf2::Transform transform(tf_rot, tf_orig);

          //Distance from camera to aruco
          //marker_i.distance = tf2Sqrt(tf2::tf2Dot(tf_orig, tf_orig));
          marker_i.distance = 0.0;

          //std::cout << transform << std::endl;
          std::cout << "camera2marker tf: " << "x: ";
          std::cout << transform.getOrigin().getX() << " y: ";
          std::cout << transform.getOrigin().getY() << " z: ";
          std::cout << transform.getOrigin().getZ() << " X: ";
          std::cout << transform.getRotation().getX() << " Y: ";
          std::cout << transform.getRotation().getY() << " Z: ";
          std::cout << transform.getRotation().getZ() << " W: ";
          std::cout << transform.getRotation().getW() << std::endl;

          //cameraToMarker Reference
          transform = cameraToReference_tf*transform;

          geometry_msgs::msg::TransformStamped tf_msg;
	        tf_msg.header.stamp = curr_stamp;
          tf_msg.header.frame_id = reference_frame_;
          tf_msg.child_frame_id = topic_ + "_marker_frame_" + std::to_string(marker_ids[i]);
          tf_msg.transform = tf2::toMsg(transform);
          tf_broadcaster->sendTransform(tf_msg);

          //cv::drawFrameAxes(inImage_copy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.05);
          aruco_ros::draw_axis(inImage_copy, cameraMatrix, distCoeffs, ret_ros, tvecs[i], marker_size_/2 , marker_size_);
          

	        tf2::toMsg(transform, marker_i.pose.pose);
          marker_i.header.frame_id = reference_frame_;
          

        }
      }
      {
        // publish marker array
        if (marker_msg_->markers.size() > 0) {
          marker_pub_->publish(*marker_msg_);
        }
      }

      {
        marker_list_msg_.data.resize(markers_.size());
        for (std::size_t i = 0; i < markers_.size(); ++i) {
          marker_list_msg_.data[i] = markers_[i].id;
        }

        marker_list_pub_->publish(marker_list_msg_);
      }
/*
      // draw detected markers on the image for visualization
      for (std::size_t i = 0; i < markers_.size(); ++i) {
        markers_[i].draw(inImage_, cv::Scalar(0, 0, 255), 2);
      }

      // draw a 3D cube in each marker if there is 3D info
      if (camParam_.isValid() && marker_size_ > 0) {
        for (std::size_t i = 0; i < markers_.size(); ++i) {
          aruco::CvDrawingUtils::draw3dAxis(inImage_, markers_[i], camParam_);
        }
      }
*/
      // publish input image with markers drawn on it
      if (publishImage) {
        // show input with augmented information
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        //out_msg.header.frame_id = camera_frame_;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage_copy;
        image_pub_.publish(out_msg.toImageMsg());
      }

      // publish image after internal image processing
      if (publishDebug) {
        // show also the internal image resulting from the threshold operation
        cv_bridge::CvImage debug_msg;
        debug_msg.header.stamp = curr_stamp;
        //debug_msg.header.frame_id = camera_frame_;
        debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
        //debug_msg.image = mDetector_.getThresholdedImage();
        debug_msg.image = inImage_copy;
        debug_pub_.publish(debug_msg.toImageMsg());
      }
    } catch (cv_bridge::Exception & e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  void cam_info_callback(sensor_msgs::msg::CameraInfo::ConstPtr msg)
  {
  if (useCamInfo_) {

    int dist_dim = msg->d.size();

    std::cout << "dist_dim: " << " ";
    std::cout << dist_dim << std::endl;

    cameraMatrix = cv::Mat::zeros(3, 3, CV_32FC1);
    distCoeffs = cv::Mat::zeros(dist_dim, 1, CV_32FC1);

    int index = 0;
    for (int i = 0; i < cameraMatrix.cols; i++) {
        for (int j = 0; j < cameraMatrix.rows; j++) {
            cameraMatrix.at<float>(i, j) = msg->k[index];
            index++;
        }
    }

    for (size_t i = 0; i < dist_dim; ++i) {
        distCoeffs.at<float>(i, 0) = msg->d[i];
    }
    
    //cam_info_received = true;
  }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::shared_ptr<ArucoMarkerPublisher> marker_pub = std::make_shared<ArucoMarkerPublisher>(rclcpp::NodeOptions());
  //marker_pub->setup();
  rclcpp::spin(marker_pub->get_node_base_interface());
  rclcpp::shutdown();
}