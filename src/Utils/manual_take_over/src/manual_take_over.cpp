#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <std_msgs/msg/empty.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <sensor_msgs/msg/joy.hpp>


using namespace std;
using namespace std::chrono_literals;

class ManualTakeOver : public rclcpp::Node
{
public:
  ManualTakeOver() : Node("manual_take_over")
  {
    mandatory_stop_pub_ = this->create_publisher<std_msgs::msg::Empty>("/mandatory_stop_to_planner", 10);
    cmd_pub_ = this->create_publisher<quadrotor_msgs::msg::PositionCommand>("/position_cmd", 10);
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "/joystick_from_bridge", 10,
      std::bind(&ManualTakeOver::joy_sub_cb, this, std::placeholders::_1));
    cmd_sub_ = this->create_subscription<quadrotor_msgs::msg::PositionCommand>(
      "/position_cmd", 100,
      std::bind(&ManualTakeOver::position_cmd_sub_cb, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      400ms, std::bind(&ManualTakeOver::timer_callback, this));
  }

private:
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr mandatory_stop_pub_;
  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time recv_joy_time_;
  bool flag_recv_joy_ = false;

  rclcpp::Time recv_cmd_time_;
  bool flag_recv_cmd_ = false;
  bool flag_planner_stop_cmds_ = false;

  bool flag_mandatory_stoped_ = false;

  Eigen::Vector4d cur_drone_pos;

  sensor_msgs::msg::Joy joy_;

  void joy_sub_cb(const sensor_msgs::msg::Joy::ConstSharedPtr &msg)
  {
    rclcpp::Time now = this->now();
    flag_recv_joy_ = true;
    recv_joy_time_ = now;
    joy_ = *msg;

    if (msg->buttons[0] || msg->buttons[1] || msg->buttons[2] || msg->buttons[3])
    {
      flag_mandatory_stoped_ = true;
      std_msgs::msg::Empty stop_msg;
      mandatory_stop_pub_->publish(stop_msg);
    }

    if (flag_mandatory_stoped_ && flag_planner_stop_cmds_)
    {
      constexpr double MAX_VEL = 0.2;
      static bool have_last_cmd = false;
      static rclcpp::Time last_cmd_t;
      static Eigen::Vector4d last_cmd;

      if (!have_last_cmd)
      {
        have_last_cmd = true;
        last_cmd_t = now;
        last_cmd = cur_drone_pos;
      }

      quadrotor_msgs::msg::PositionCommand cmd_msg;
      cmd_msg.header.stamp = now;
      cmd_msg.header.frame_id = "manual_take_over";
      double delta_t = (now - last_cmd_t).seconds();
      last_cmd(0) += joy_.axes[4] * MAX_VEL * delta_t;
      last_cmd(1) += joy_.axes[3] * MAX_VEL * delta_t;
      last_cmd(2) += joy_.axes[1] * MAX_VEL * delta_t;
      last_cmd(3) += joy_.axes[0] * MAX_VEL * delta_t;
      cmd_msg.position.x = last_cmd(0);
      cmd_msg.position.y = last_cmd(1);
      cmd_msg.position.z = last_cmd(2);
      cmd_msg.yaw = last_cmd(3);
      cmd_msg.velocity.x = joy_.axes[4] * MAX_VEL;
      cmd_msg.velocity.y = joy_.axes[3] * MAX_VEL;
      cmd_msg.velocity.z = joy_.axes[1] * MAX_VEL;
      cmd_pub_->publish(cmd_msg);

      last_cmd_t = now;
    }
  }

  // #      ^                ^
  // #    +1|              +4|
  // # <-+0      ->     <-+3      ->
  // #      |                |
  // #      V                V

  void position_cmd_sub_cb(const quadrotor_msgs::msg::PositionCommand::ConstSharedPtr &msg)
  {
    if (msg->header.frame_id != string("manual_take_over"))
    {
      flag_recv_cmd_ = true;
      recv_cmd_time_ = this->now();
    }

    cur_drone_pos(0) = msg->position.x;
    cur_drone_pos(1) = msg->position.y;
    cur_drone_pos(2) = msg->position.z;
    cur_drone_pos(3) = msg->yaw;
  }

  void timer_callback()
  {
    rclcpp::Time t_now = this->now();

    constexpr double TIME_OUT = 1.0;
    if (flag_recv_joy_ && (t_now - recv_joy_time_).seconds() > TIME_OUT)
    {
      RCLCPP_ERROR(this->get_logger(), "Lost manual take over joystick messages!");
    }

    if (flag_recv_cmd_ && (t_now - recv_cmd_time_).seconds() > TIME_OUT)
    {
      flag_planner_stop_cmds_ = true;
    }
    else
    {
      flag_planner_stop_cmds_ = false;
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManualTakeOver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
