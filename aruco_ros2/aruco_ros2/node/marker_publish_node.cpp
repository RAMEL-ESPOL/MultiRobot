/*****************************
Detect and Draw Aruco Markers Node.
********************************/

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "aruco_ros/marker_publish.hpp"

/**
 * @brief main function
 */
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoMarkerPublisher>(rclcpp::NodeOptions());

  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
}
