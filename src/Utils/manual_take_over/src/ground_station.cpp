#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <sensor_msgs/msg/joy.hpp>


using namespace std;

class ManualTakeOverStation : public rclcpp::Node
{
public:
  ManualTakeOverStation() : Node("manual_take_over_ground_station")
  {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joy", 10, std::bind(&ManualTakeOverStation::joy_sub_cb, this, std::placeholders::_1));
    joy_pub_ = this->create_publisher<sensor_msgs::msg::Joy>("/joystick_from_users", 10);
  }

private:
  bool flag_mandatory_stoped_ = false;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  void joy_sub_cb(const sensor_msgs::msg::Joy::ConstSharedPtr &msg)
  {
    if (msg->buttons[0] || msg->buttons[1] || msg->buttons[2] || msg->buttons[3])
    {
      flag_mandatory_stoped_ = true;
    }

    if (flag_mandatory_stoped_)
    {
      joy_pub_->publish(*msg);
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManualTakeOverStation>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
