#include <rclcpp/rclcpp.hpp>
#include "drone_detector/drone_detector.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("drone_detect");

  detect::DroneDetector drone_detector(node);
  drone_detector.test();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
