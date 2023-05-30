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

#include <memory>
#include <vector>
#include <string>
#include <map>
#include "aruco_ros/test_simplesingle_aruco_ros2.hpp"


using std::experimental::filesystem::current_path;

TEST(test_simplesingle_aruco_ros2, test_caminfo)
{
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info =
    std::make_shared<sensor_msgs::msg::CameraInfo>();
  camera_info->height = 720;
  camera_info->width = 1280;
  camera_info->distortion_model = "plumb_bob";
  array<double,
    9> veck = {783.8192972, 0.0, 649.1396957, 0.0, 786.38339479, 345.07377121, 0.0, 0.0, 1.0};
  camera_info->k = veck;
  vector<double> vecd = {0.10258321, -0.2406874, -0.00259027, -0.00244419, 0.11205082};
  camera_info->d = vecd;
  array<double, 9> vecr = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camera_info->r = vecr;
  array<double,
    12> vecp =
  {852.395142, 0.0, 565.89763, 0.0, 0.0, 922.066223, 386.58625, 0.0, 0.0, 0.0, 1.0, 0.0};
  camera_info->p = vecp;


  std::vector<rclcpp::Parameter> initial_parameters =
  {rclcpp::Parameter("refinementMethod", std::string("HARRIS"))};
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.parameter_overrides(initial_parameters);
  auto node = std::make_shared<ArucoSimple>(node_options);
  //   std::vector<rclcpp::Parameter> initial_parameters =
  //     {rclcpp::Parameter("refinementMethod", std::string("HARRIS"))};
  // node->set_parameters(initial_parameters);
  node->useRectifiedImages = false;
  node->cam_info_callback(camera_info);
  EXPECT_EQ(node->cam_info_received, true);
  EXPECT_EQ((int)node->camParam.CamSize.width, (int)camera_info->width);
  EXPECT_EQ((int)node->camParam.CamSize.height, (int)camera_info->height);

  EXPECT_EQ(node->camParam.CameraMatrix.at<float>(0, 0), (float)camera_info->k[0]);
  EXPECT_EQ(node->camParam.CameraMatrix.at<float>(1, 1), (float)camera_info->k[4]);
  EXPECT_EQ(node->camParam.CameraMatrix.at<float>(0, 2), (float)camera_info->k[2]);
  EXPECT_EQ(node->camParam.CameraMatrix.at<float>(1, 2), (float)camera_info->k[5]);
}
TEST(test_simplesingle_aruco_ros2, test_caminfo2)
{
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info =
    std::make_shared<sensor_msgs::msg::CameraInfo>();
  camera_info->height = 720;
  camera_info->width = 1280;
  camera_info->distortion_model = "plumb_bob";
  array<double,
    9> veck = {783.8192972, 0.0, 649.1396957, 0.0, 786.38339479, 345.07377121, 0.0, 0.0, 1.0};
  camera_info->k = veck;
  vector<double> vecd = {0.10258321, -0.2406874, -0.00259027, -0.00244419, 0.11205082};
  camera_info->d = vecd;
  array<double, 9> vecr = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camera_info->r = vecr;
  array<double,
    12> vecp =
  {852.395142, 0.0, 565.89763, 0.0, 0.0, 922.066223, 386.58625, 0.0, 0.0, 0.0, 1.0, 0.0};
  camera_info->p = vecp;


  std::vector<rclcpp::Parameter> initial_parameters =
  {rclcpp::Parameter("refinementMethod", std::string("NONE"))};
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.parameter_overrides(initial_parameters);
  auto node = std::make_shared<ArucoSimple>(node_options);
  //   std::vector<rclcpp::Parameter> initial_parameters =
  //     {rclcpp::Parameter("refinementMethod", std::string("HARRIS"))};
  // node->set_parameters(initial_parameters);
  node->useRectifiedImages = false;
  node->cam_info_callback(camera_info);
  EXPECT_EQ(node->cam_info_received, true);
  EXPECT_EQ((int)node->camParam.CamSize.width, (int)camera_info->width);
  EXPECT_EQ((int)node->camParam.CamSize.height, (int)camera_info->height);

  EXPECT_EQ(node->camParam.CameraMatrix.at<float>(0, 0), (float)camera_info->k[0]);
  EXPECT_EQ(node->camParam.CameraMatrix.at<float>(1, 1), (float)camera_info->k[4]);
  EXPECT_EQ(node->camParam.CameraMatrix.at<float>(0, 2), (float)camera_info->k[2]);
  EXPECT_EQ(node->camParam.CameraMatrix.at<float>(1, 2), (float)camera_info->k[5]);
}
TEST(test_simplesingle_aruco_ros2, test_caminfo3)
{
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info =
    std::make_shared<sensor_msgs::msg::CameraInfo>();
  camera_info->height = 720;
  camera_info->width = 1280;
  camera_info->distortion_model = "plumb_bob";
  array<double,
    9> veck = {783.8192972, 0.0, 649.1396957, 0.0, 786.38339479, 345.07377121, 0.0, 0.0, 1.0};
  camera_info->k = veck;
  vector<double> vecd = {0.10258321, -0.2406874, -0.00259027, -0.00244419, 0.11205082};
  camera_info->d = vecd;
  array<double, 9> vecr = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camera_info->r = vecr;
  array<double,
    12> vecp =
  {852.395142, 0.0, 565.89763, 0.0, 0.0, 922.066223, 386.58625, 0.0, 0.0, 0.0, 1.0, 0.0};
  camera_info->p = vecp;


  std::vector<rclcpp::Parameter> initial_parameters =
  {rclcpp::Parameter("refinementMethod", std::string("SUBPIX"))};
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.parameter_overrides(initial_parameters);
  auto node = std::make_shared<ArucoSimple>(node_options);
  //   std::vector<rclcpp::Parameter> initial_parameters =
  //     {rclcpp::Parameter("refinementMethod", std::string("HARRIS"))};
  // node->set_parameters(initial_parameters);
  node->useRectifiedImages = false;
  node->cam_info_callback(camera_info);
  EXPECT_EQ(node->cam_info_received, true);
  EXPECT_EQ((int)node->camParam.CamSize.width, (int)camera_info->width);
  EXPECT_EQ((int)node->camParam.CamSize.height, (int)camera_info->height);

  EXPECT_EQ(node->camParam.CameraMatrix.at<float>(0, 0), (float)camera_info->k[0]);
  EXPECT_EQ(node->camParam.CameraMatrix.at<float>(1, 1), (float)camera_info->k[4]);
  EXPECT_EQ(node->camParam.CameraMatrix.at<float>(0, 2), (float)camera_info->k[2]);
  EXPECT_EQ(node->camParam.CameraMatrix.at<float>(1, 2), (float)camera_info->k[5]);
}
TEST(test_simplesingle_aruco_ros2, test_aruco_detected_ID)
{
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info =
    std::make_shared<sensor_msgs::msg::CameraInfo>();
  camera_info->height = 720;
  camera_info->width = 1280;
  camera_info->distortion_model = "plumb_bob";
  array<double,
    9> veck = {783.8192972, 0.0, 649.1396957, 0.0, 786.38339479, 345.07377121, 0.0, 0.0, 1.0};
  camera_info->k = veck;
  vector<double> vecd = {0.10258321, -0.2406874, -0.00259027, -0.00244419, 0.11205082};
  camera_info->d = vecd;
  array<double, 9> vecr = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camera_info->r = vecr;
  array<double,
    12> vecp =
  {852.395142, 0.0, 565.89763, 0.0, 0.0, 922.066223, 386.58625, 0.0, 0.0, 0.0, 1.0, 0.0};
  camera_info->p = vecp;

  auto node = std::make_shared<ArucoSimple>(rclcpp::NodeOptions());
  node->marker_size = 0.1;
  node->cam_info_callback(camera_info);
  EXPECT_EQ(node->cam_info_received, true);
  string path_to_test_image1 = "test/1test.jpg";
  cv::Mat rgb_frame = cv::imread(path_to_test_image1, cv::IMREAD_COLOR);
  sensor_msgs::msg::Image::SharedPtr input_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rgb_frame).toImageMsg();
  node->image_callback(input_msg);

  EXPECT_EQ(node->markers.size(), 1);
  EXPECT_EQ(node->markers[0].id, 590);
}


TEST(test_simplesingle_aruco_ros2, test_aruco_detected_multiple_id)
{
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info =
    std::make_shared<sensor_msgs::msg::CameraInfo>();
  camera_info->height = 720;
  camera_info->width = 1280;
  camera_info->distortion_model = "plumb_bob";
  array<double,
    9> veck = {783.8192972, 0.0, 649.1396957, 0.0, 786.38339479, 345.07377121, 0.0, 0.0, 1.0};
  camera_info->k = veck;
  vector<double> vecd = {0.10258321, -0.2406874, -0.00259027, -0.00244419, 0.11205082};
  camera_info->d = vecd;
  array<double, 9> vecr = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camera_info->r = vecr;
  array<double,
    12> vecp =
  {852.395142, 0.0, 565.89763, 0.0, 0.0, 922.066223, 386.58625, 0.0, 0.0, 0.0, 1.0, 0.0};
  camera_info->p = vecp;

  auto node = std::make_shared<ArucoSimple>(rclcpp::NodeOptions());
  node->cam_info_callback(camera_info);
  EXPECT_EQ(node->cam_info_received, true);
  string path_to_test_image2 = "test/71test.jpg";
  cv::Mat rgb_frame = cv::imread(path_to_test_image2, cv::IMREAD_COLOR);
  sensor_msgs::msg::Image::SharedPtr input_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rgb_frame).toImageMsg();
  node->image_callback(input_msg);

  EXPECT_EQ(node->markers.size(), 4);
  vector<int> anw = {4, 12, 1011, 1019};

  for (int i = 0; node->markers.size() > i; i++) {
    anw.erase(std::remove(anw.begin(), anw.end(), node->markers[i].id), anw.end());
  }
  EXPECT_EQ(anw.size(), 0);
}
TEST(test_simplesingle_aruco_ros2, test_aruco_detected_position1)
{
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info =
    std::make_shared<sensor_msgs::msg::CameraInfo>();
  camera_info->height = 720;
  camera_info->width = 1280;
  camera_info->distortion_model = "plumb_bob";
  array<double,
    9> veck = {783.8192972, 0.0, 649.1396957, 0.0, 786.38339479, 345.07377121, 0.0, 0.0, 1.0};
  camera_info->k = veck;
  vector<double> vecd = {0.10258321, -0.2406874, -0.00259027, -0.00244419, 0.11205082};
  camera_info->d = vecd;
  array<double, 9> vecr = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camera_info->r = vecr;
  array<double,
    12> vecp =
  {852.395142, 0.0, 565.89763, 0.0, 0.0, 922.066223, 386.58625, 0.0, 0.0, 0.0, 1.0, 0.0};
  camera_info->p = vecp;

  std::vector<rclcpp::Parameter> initial_parameters =
  {rclcpp::Parameter("refinementMethod", std::string("SUBPIX")),
    rclcpp::Parameter("marker_id", 590),
    rclcpp::Parameter("reference_frame", std::string(""))};
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.parameter_overrides(initial_parameters);
  auto node = std::make_shared<ArucoSimple>(node_options);
  node->cam_info_callback(camera_info);
  EXPECT_EQ(node->cam_info_received, true);
  string path_to_test_image1 = "test/1test.jpg";
  cv::Mat rgb_frame = cv::imread(path_to_test_image1, cv::IMREAD_COLOR);
  sensor_msgs::msg::Image::SharedPtr input_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rgb_frame).toImageMsg();
  node->image_callback(input_msg);

  EXPECT_EQ(node->markers.size(), 1);

  // cv::Mat answertvec = (cv::Mat_<float>(3, 1) << -0.46442503,
  //   0.035325821,
  //   1.3718389);

  // cv::Mat different = node->markers_[0].Tvec - answertvec;
  // float p = cv::norm(different);
  // EXPECT_EQ((p < 0.01), true);
}
TEST(test_simplesingle_aruco_ros2, test_aruco_detected_position2)
{
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info =
    std::make_shared<sensor_msgs::msg::CameraInfo>();
  camera_info->height = 720;
  camera_info->width = 1280;
  camera_info->distortion_model = "plumb_bob";
  array<double,
    9> veck = {783.8192972, 0.0, 649.1396957, 0.0, 786.38339479, 345.07377121, 0.0, 0.0, 1.0};
  camera_info->k = veck;
  vector<double> vecd = {0.10258321, -0.2406874, -0.00259027, -0.00244419};
  camera_info->d = vecd;
  array<double, 9> vecr = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camera_info->r = vecr;
  array<double,
    12> vecp =
  {852.395142, 0.0, 565.89763, 0.0, 0.0, 922.066223, 386.58625, 0.0, 0.0, 0.0, 1.0, 0.0};
  camera_info->p = vecp;

  std::vector<rclcpp::Parameter> initial_parameters =
  {rclcpp::Parameter("refinementMethod", std::string("HARRIS")),
    rclcpp::Parameter("marker_id", 590),
    rclcpp::Parameter("reference_frame", std::string("testframe"))};
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);
  node_options.parameter_overrides(initial_parameters);
  auto node = std::make_shared<ArucoSimple>(node_options);
  node->cam_info_callback(camera_info);
  EXPECT_EQ(node->cam_info_received, true);
  string path_to_test_image1 = "test/1test.jpg";
  cv::Mat rgb_frame = cv::imread(path_to_test_image1, cv::IMREAD_COLOR);
  sensor_msgs::msg::Image::SharedPtr input_msg =
    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rgb_frame).toImageMsg();
  node->image_callback(input_msg);

  // EXPECT_EQ(node->markers.size(), 1);

  // cv::Mat answertvec = (cv::Mat_<float>(3, 1) << -0.46442503,
  //   0.035325821,
  //   1.3718389);

  // cv::Mat different = node->markers_[0].Tvec - answertvec;
  // float p = cv::norm(different);
  // EXPECT_EQ((p < 0.01), true);
}

TEST(test_simplesingle_aruco_ros2, test_aruco_detected_position3)
{
  sensor_msgs::msg::CameraInfo::SharedPtr camera_info =
    std::make_shared<sensor_msgs::msg::CameraInfo>();
  camera_info->height = 720;
  camera_info->width = 1280;
  camera_info->distortion_model = "plumb_bob";
  array<double,
    9> veck = {783.8192972, 0.0, 649.1396957, 0.0, 786.38339479, 345.07377121, 0.0, 0.0, 1.0};
  camera_info->k = veck;
  array<double, 9> vecr = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  camera_info->r = vecr;
  array<double,
    12> vecp =
  {852.395142, 0.0, 565.89763, 0.0, 0.0, 922.066223, 386.58625, 0.0, 0.0, 0.0, 1.0, 0.0};
  camera_info->p = vecp;
  for (int k1 = 0; k1 < 2; k1++) {
    if (k1 == 0) {
      vector<double> vecd = {0.10258321, -0.2406874, -0.00259027, -0.00244419};
      camera_info->d = vecd;
    } else {
      vector<double> vecd = {0.10258321, -0.2406874, -0.00259027, -0.00244419, 0.11205082};
      camera_info->d = vecd;
    }
    for (int k2 = 0; k2 < 5; k2++) {
      std::string refinementMethod;
      switch (k1) {
        case 0:
          refinementMethod = "SUBPIX";
          break;
        case 1:
          refinementMethod = "HARRIS";
          break;
        case 2:
          refinementMethod = "NONE";
          break;
        case 3:
          refinementMethod = "LINES";
          break;
        case 4:
          refinementMethod = "";
          break;
      }
      for (int k3 = 0; k3 < 2; k3++) {
        bool useRectifiedImages = (k3 == 0);
        for (int k4 = 0; k4 < 2; k4++) {
          std::string reference_frame;
          if (k4 == 0) {
            reference_frame = "testframe";
          } else {
            reference_frame = "";
          }

          for (int k5 = 0; k5 < 2; k5++) {
            int marker_id;
            if (k5 == 0) {
              marker_id = 590;
            } else {
              marker_id = 582;
            }

            std::vector<rclcpp::Parameter> initial_parameters =
            {rclcpp::Parameter("refinementMethod", refinementMethod),
              rclcpp::Parameter("marker_id", marker_id),
              rclcpp::Parameter("reference_frame", reference_frame),
              rclcpp::Parameter("image_is_rectified", useRectifiedImages)};
            rclcpp::NodeOptions node_options;
            node_options.allow_undeclared_parameters(true);
            node_options.automatically_declare_parameters_from_overrides(true);
            node_options.parameter_overrides(initial_parameters);
            auto node = std::make_shared<ArucoSimple>(node_options);
            node->cam_info_callback(camera_info);
            EXPECT_EQ(node->cam_info_received, true);
            string path_to_test_image1 = "test/1test.jpg";
            cv::Mat rgb_frame = cv::imread(path_to_test_image1, cv::IMREAD_COLOR);
            sensor_msgs::msg::Image::SharedPtr input_msg =
              cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", rgb_frame).toImageMsg();
            node->image_callback(input_msg);
          }
        }
      }
    }
  }


  // EXPECT_EQ(node->markers.size(), 1);

  // cv::Mat answertvec = (cv::Mat_<float>(3, 1) << -0.46442503,
  //   0.035325821,
  //   1.3718389);

  // cv::Mat different = node->markers_[0].Tvec - answertvec;
  // float p = cv::norm(different);
  // EXPECT_EQ((p < 0.01), true);
}
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
