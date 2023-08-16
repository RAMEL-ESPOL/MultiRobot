/*
\section Final Notes

 - REQUIREMENTS: OpenCv >= 2.1.0. and OpenGL for (aruco_test_gl and aruco_test_board_gl)
 - CONTACT: Rafael Munoz-Salinas: rmsalinas@uco.es
 - This libary is free software and come with no guaratee!
 
*/

#ifndef ARUCO_ROS__ARUCO_ROS_UTILS_HPP_
#define ARUCO_ROS__ARUCO_ROS_UTILS_HPP_

#include "tf2/LinearMath/Transform.h"
#include "tf2/transform_datatypes.h"
#include "aruco/aruco.h"
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/camera_info.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


/// @brief Package with complementary functions
namespace aruco_ros
{

/// @brief convert the marker orientation vector wrt camera for ROS
/// @param Rvec Axis with angle magnitude (radians) vector of the marker
/// @return rotation vector wrt camera for ROS
cv::Vec3d rotationVectorWithROSAxes(cv::Vec3d &Rvec);

/// @brief draw the axes' markers in a image.
/// @param image Image captured by the camera
/// @param cameraMatrix Focal and optical information of the camera
/// @param distCoeffs Information of the distortion of the image captured
/// @param rvec Rotation vector
/// @param tvec Translation vector
/// @param length of the axes
/// @param markerLength real size of the marker
/// @param thickness of the axes' lines
void draw_axis(const cv::Mat  &image, const cv::InputArray &cameraMatrix,
               const cv::InputArray &distCoeffs, const cv::InputArray &rvec,
               const cv::InputArray &tvec, float length, float markerLength, int thickness = 2);


/// @brief convert the marker orientation vector wrt camera (image public)
/// @param Rvec Axis with angle magnitude (radians) vector of the marker
/// @return rotation vector wrt camera (image public)
cv::Vec3d rotationVectorwrtCamera(cv::Vec3d &Rvec);

/**
   * @brief rosCameraInfo2ArucoCamParams gets the camera intrinsics from a CameraInfo message and copies them
   *                                     to aruco_ros own data structure
   * @param cam_info
   * @param useRectifiedParameters if true, the intrinsics are taken from cam_info.P and the distortion parameters
   *                               are set to 0. Otherwise, cam_info.K and cam_info.D are taken.
   * @return return camera matrix or parameter
   */


aruco::CameraParameters rosCameraInfo2ArucoCamParams(
  const sensor_msgs::msg::CameraInfo & cam_info,
  bool useRectifiedParameters);


/**
 * @brief arucoMarker2Tf convert the marker pose info to TF2 transform type
 * @param marker   input marker
 * @param rotate_marker_axis if true, Rotate axis direction as to fit from opencv to ROS frame.
 * @return transformation of marker in type TF2
 */


tf2::Transform arucoMarker2Tf(const aruco::Marker & marker, bool rotate_marker_axis = true);




} 
 // namespace aruco_ros
#endif  // ARUCO_ROS__ARUCO_ROS_UTILS_HPP_
