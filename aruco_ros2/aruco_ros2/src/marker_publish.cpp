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

#include <string>
#include <memory>
#include "aruco_ros/marker_publish.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <tf2/convert.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
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
  

  get_parameter_or("use_camera_info", useCamInfo_, true);
  declare_parameter<std::string>("aruco_dictionary_id", "DICT_4X4_250");
  std::string dictionary_id_name = get_parameter("aruco_dictionary_id").as_string();

  // Set up subscriptions
  image_sub_ =
    image_transport::create_subscription(
    this, "/image",
    std::bind(&ArucoMarkerPublisher::image_callback, this, std::placeholders::_1), "raw");
  cam_info_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera_info",
    rclcpp::SensorDataQoS(),
    std::bind(&ArucoMarkerPublisher::cam_info_callback, this, std::placeholders::_1));
  if (useCamInfo_) {
    get_parameter_or("marker_size", marker_size_, 0.026);
    //get_parameter_or("image_is_rectified", useRectifiedImages_, false);
    get_parameter_or("reference_frame", reference_frame_, string{"camera_color_frame"});
    get_parameter_or("camera_frame", camera_frame_, string{"camera_color_optical_frame"});
    //get_parameter_or("rotate_marker_axis", rotate_marker_axis_, false);
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

  // Make sure we have a valid dictionary id
  /*  try {
      dictionary_id_ = cv::aruco::getPredefinedDictionary(cv::aruco::__getattribute__(dictionary_id_name));
      if (dictionary_id_.empty() || dictionary_id_.total() == 0) {
        throw std::runtime_error("");
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "bad aruco_dictionary_id: %s", dictionary_id_name.c_str());
      std::string options;
      const auto dict_attrs = cv::aruco:dictionary:__getattr__();
      for (const auto& attr : dict_attrs) {
        if (attr.find("DICT") == 0) {
          options += attr + "\n";
        }
      }
      RCLCPP_ERROR(get_logger(), "valid options:\n%s", options.c_str());
      throw;
    }*/

  //std::shared_ptr<cv_bridge::CvBridge> bridge_;
  //bridge_ = std::make_shared<cv_bridge::CvBridge>();
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
      inImage_.copyTo(inImage_copy);

      // clear out previous detection results
      markers_.clear();
      
      std::vector<std::vector<cv::Point2f>> corners, rejected;
      std::vector<int> marker_ids;

      //std::vector<cv::Point2f> rejected;

      
      //cv::Ptr<cv::aruco::DetectorParameters> aruco_parameters_ = cv::aruco::DetectorParameters::create();
      
      // Ok, let's detect
      //mDetector_.detect(inImage_, markers_, camParam_, marker_size_, false);
      
      //cv::Mat cameraMatrix, distCoeffs;

      cv::Mat cameraMatrix = cv::Mat::zeros(3, 3, CV_32FC1);
      cv::Mat distCoeffs(5, 1, CV_32FC1);

      // Custom values for the matrix
      float values[] = {1364.8021240234375, 0.0, 973.9801025390625, 0.0, 1365.6466064453125, 551.3172607421875, 0.0, 0.0, 1.0};

      // Fill the matrix with custom values
      int index = 0;
      for (int i = 0; i < cameraMatrix.cols; i++) {
          for (int j = 0; j < cameraMatrix.rows; j++) {
              cameraMatrix.at<float>(i, j) = values[index];
              index++;
          }
      }

      float values2[] = {0.1670105755329132, -0.5500425100326538, -0.0003902913013007492, -0.0008471994078718126, 0.49133142828941345};
      int index2 = 0;
      for (int i = 0; i < distCoeffs.rows; i++) {
          for (int j = 0; j < distCoeffs.cols; j++) {
              distCoeffs.at<float>(i, j) = values2[index2];
              index2++;
          }
      }

      //cv::Size size;
      //aruco::CameraParameters(cameraMatrix, distCoeffs, size);
/*
      cv::FileStorage fs("calibration.yaml", cv::FileStorage::READ);
      if (fs.isOpened()){
          std::cout << "SIIIIIUUUUUUUUU" << std::endl;
      }
      fs["camera_matrix"] >> cameraMatrix;
      fs["distortion_coefficients"] >> distCoeffs;
*/

      std::cout << "Camera Matrix:" << std::endl;
      for (int i = 0; i < cameraMatrix.rows; i++) {
          for (int j = 0; j < cameraMatrix.cols; j++) {
              std::cout << cameraMatrix.at<float>(i, j) << " ";
          }
          std::cout << std::endl;
      }

      std::cout << "Distortion Coefficients:" << std::endl;
      for (int i = 0; i < distCoeffs.rows; i++) {
          for (int j = 0; j < distCoeffs.cols; j++) {
              std::cout << distCoeffs.at<float>(i, j) << " ";
          }
          std::cout << std::endl;
      }


      //cv::FileStorage fs("calibration.yaml", cv::FileStorage::READ);
      //if (!fs.isOpened())
          //return false;
      //fs["camera_matrix"] >> cameraMatrix;
      //fs["distortion_coefficients"] >> distCoeffs;
      //return true;

      //cv::Mat thres2
      //inImage_.copyTo(thres2);
      cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
      cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
      cv::aruco::detectMarkers(inImage_, dictionary, corners, marker_ids, detectorParams, rejected);
      if  (marker_ids.size() > 0) {
        int nMarkers = marker_ids.size();
        std::vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::drawDetectedMarkers(inImage_copy, corners, marker_ids);
        cv::aruco::estimatePoseSingleMarkers(corners, marker_size_, cameraMatrix, distCoeffs, rvecs, tvecs);
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

          cv::aruco::drawAxis(inImage_copy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.05);

        }

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
          //tf2::Transform transform = aruco_ros::arucoMarker2Tf(markers_[i], rotate_marker_axis_);

          /*std::cout << transform.getOrigin().getX() << " || " << transform.getOrigin().getY() <<
            " || " << transform.getOrigin().getZ() << endl;
          transform = cameraToReference_tf * transform;*/
          
          //RotationAndTranslationVectorsToTransform
          cv::Mat rot_mat(3, 3, CV_64F);
          cv::Rodrigues(aruco_ros::rotationVectorWithROSAxes(rvecs[i]), rot_mat);
          tf2::Matrix3x3 tf_rot(
              rot_mat.at<double>(0, 0), rot_mat.at<double>(0, 1), rot_mat.at<double>(0, 2),
              rot_mat.at<double>(1, 0), rot_mat.at<double>(1, 1), rot_mat.at<double>(1, 2),
              rot_mat.at<double>(2, 0), rot_mat.at<double>(2, 1), rot_mat.at<double>(2, 2));
          tf2::Vector3 tf_orig(-tvecs[i][1], -tvecs[i][2], tvecs[i][0]);
          tf2::Transform transform(tf_rot, tf_orig);

          //cameraToMarker Reference
          transform = cameraToReference_tf*transform;

          geometry_msgs::msg::TransformStamped tf_msg;
	        tf_msg.header.stamp = curr_stamp;
          tf_msg.header.frame_id = camera_frame_;
          tf_msg.child_frame_id = "marker_frame_" + std::to_string(marker_ids[i]);
          tf_msg.transform = tf2::toMsg(transform);
          tf_broadcaster->sendTransform(tf_msg);
          
          /*
          tf_msg.transform.translation.x = tvecs[i][0];
          tf_msg.transform.translation.y = tvecs[i][1];
          tf_msg.transform.translation.z = tvecs[i][2];

          cv::Mat rot_mat(3, 3, CV_64F);
          cv::Rodrigues(rvecs[i], rot_mat);
          cv::Mat rotate_to_ros =
              (cv::Mat_<double>(3, 3) << 0, 0, 1, 0, -1, 0, 1, 0, 0);
          rot_mat = rot_mat * rotate_to_ros.t();
          cv::Vec3d ret;
          cv::Rodrigues(rot_mat, ret);
          
          
          cv::Matx33d rotation_matrix(rot_mat);
          */
          //cv::Mat rotationTransposed = rot_mat.t();
/*
          tf2::Matrix3x3 tf2_matrix(
              rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2),
              rotation_matrix(1, 0), rotation_matrix(1, 1), rotation_matrix(1, 2),
              rotation_matrix(2, 0), rotation_matrix(2, 1), rotation_matrix(2, 2)
          );*/
          /*
          cv::Vec3d rotation_vector;
          cv::Rodrigues(rotation_matrix, rotation_vector);

          tf2::Quaternion q;
          q.setRPY(rotation_vector[1], rotation_vector[0], rotation_vector[2]);


          tf2::Matrix3x3 tf_rot(
              rot.at<double>(0, 0), rot.at<double>(0, 1), rot.at<double>(0, 2),
              rot.at<double>(1, 0), rot.at<double>(1, 1), rot.at<double>(1, 2),
              rot.at<double>(2, 0), rot.at<double>(2, 1), rot.at<double>(2, 2));
          tf2::Vector3 tf_orig(Tvec[0], Tvec[1], Tvec[2]);
          tf2::Transform tf_msg(tf_rot, tf_orig);
          //tf2::Quaternion q;
          //tf2_matrix.getRotation(q);
          //tf2::Matrix3x3(rotation_matrix).getRotation(q);

          tf_msg.transform.rotation.x = q.getX();
          tf_msg.transform.rotation.y = q.getY();
          tf_msg.transform.rotation.z = q.getZ();
          tf_msg.transform.rotation.w = q.getW();
          tf_broadcaster->sendTransform(tf_msg);
          tf2::Transform transform;
          tf2::fromMsg(tf_msg.transform, transform);
          */
	        tf2::toMsg(transform, marker_i.pose.pose);
          marker_i.header.frame_id = reference_frame_;
          

        }
      }
      // marker array publish

      {
        /*marker_msg_->markers.clear();
        marker_msg_->markers.resize(markers_.size());
        marker_msg_->header.stamp = curr_stamp;*/
        /*
        for (size_t i = 0; i < markers_.size(); ++i) {
          aruco_msgs::msg::Marker & marker_i = marker_msg_->markers.at(i);
          marker_i.header.stamp = curr_stamp;
          marker_i.id = markers_.at(i).id;
          marker_i.confidence = 1.0;
        }*/

        // if there is camera info let's do 3D stuff
        /*if (useCamInfo_) {
          geometry_msgs::msg::TransformStamped cameraToReference;
          tf2::Transform cameraToReference_tf;

          if (reference_frame_ != camera_frame_) {
            if (!getTransform(reference_frame_, camera_frame_, cameraToReference)) {
              return;
            }
            tf2::fromMsg(cameraToReference.transform, cameraToReference_tf);
          } else {
            cameraToReference_tf.setIdentity();
          }*/

          // Now find the transform for each detected marker
          /*for (size_t i = 0; i < markers_.size(); ++i) {
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
        }*/

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
      /*if (camParam_.isValid() && marker_size_ > 0) {
        for (size_t i = 0; i < markers_.size(); ++i) {
          aruco::CvDrawingUtils::draw3dAxis(inImage_, markers_[i], camParam_);
        }
      }*/

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
        out_msg.image = inImage_copy;
        image_pub_.publish(out_msg.toImageMsg());
      }

      // publish image after internal image processing
      if (publishDebug) {
        // show also the internal image resulting from the threshold operation
        cv_bridge::CvImage debug_msg;
        debug_msg.header.stamp = curr_stamp;
        debug_msg.header.frame_id = camera_frame_;
        debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
        //debug_msg.image = mDetector_.getThresholdedImage();
        debug_msg.image = inImage_copy;
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

    //int dist_dim = msg.d.size();
/*
    cv::Mat cameraMatrix = cv::Mat::zeros(3, 3, CV_64FC1);
    cv::Mat distCoeffs(dist_dim, 1, CV_64FC1);

    cameraMatrix.at<double>(0, 0) = msg.p[0];   cameraMatrix.at<double>(0, 1) = msg.p[1];
    cameraMatrix.at<double>(0, 2) = msg.p[2];
    cameraMatrix.at<double>(1, 0) = msg.p[4];   cameraMatrix.at<double>(1, 1) = msg.p[5];
    cameraMatrix.at<double>(1, 2) = msg.p[6];
    cameraMatrix.at<double>(2, 0) = msg.p[8];   cameraMatrix.at<double>(2, 1) = msg.p[9];
    cameraMatrix.at<double>(2, 2) = msg.p[10];

    for (int i = 0; i < dist_dim; ++i) {
        distCoeffs.at<double>(i, 0) = msg.d[i];
    }
*/
    camParam_ = aruco_ros::rosCameraInfo2ArucoCamParams(*msg, useRectifiedImages_);

    cam_info_received = true;
  }
}

/*inline static bool readCameraParameters(std::string filename, cv::Mat &camMatrix, cv::Mat &distCoeffs) {
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}*/

