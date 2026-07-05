#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <Eigen/Dense>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <quadrotor_msgs/msg/takeoff_land.hpp>
#include <px4_msgs/msg/input_rc.hpp>

using namespace std;
using namespace std::chrono_literals;

class MultipointPlanNode : public rclcpp::Node
{
public:
  MultipointPlanNode() : Node("multipointplan_node")
  {
    string yaml_path;
    this->declare_parameter<string>("yaml_path", "");
    this->declare_parameter<double>("next_distance", 0.7);
    this->declare_parameter<int>("start_plan", 1);
    this->declare_parameter<int>("back_plan", 1);
    this->declare_parameter<int>("fligt_type", 1);

    yaml_path = this->get_parameter("yaml_path").as_string();
    next_distance = this->get_parameter("next_distance").as_double();
    flag_start_plan = this->get_parameter("start_plan").as_int();
    flag_back_plan = this->get_parameter("back_plan").as_int();
    fligt_type = this->get_parameter("fligt_type").as_int();

    if (yaml_path.empty() || fligt_type < 1 || fligt_type > 4)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to get parameter, please check it");
      rclcpp::shutdown();
      return;
    }

    readpyt(yaml_path);

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom_topic", 10,
      std::bind(&MultipointPlanNode::odom_goal_cb, this, std::placeholders::_1));
    if (flag_start_plan) {
      startcommand_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/move_base_simple/goal", 10,
        std::bind(&MultipointPlanNode::startplan_cb, this, std::placeholders::_1));
    }
    if (flag_back_plan) {
      backcommand_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/back_trigger", 10,
        std::bind(&MultipointPlanNode::backplan_cb, this, std::placeholders::_1));
    }
    rc_sub = this->create_subscription<px4_msgs::msg::InputRc>(
      "/fmu/in/input_rc", 10,
      std::bind(&MultipointPlanNode::rc_cb, this, std::placeholders::_1));
    takeoff_land_pub = this->create_publisher<quadrotor_msgs::msg::TakeoffLand>(
      "/px4ctrl/takeoff_land", 10);
    startcommand_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/move_base_simple/goal", 10);
    backcommand_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/back_trigger", 10);
    point_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/goal", 10);
    yaw_pub = this->create_publisher<quadrotor_msgs::msg::PositionCommand>(
      "/planning/yaw", 10);

    timer = this->create_wall_timer(
      10ms, std::bind(&MultipointPlanNode::Point_send, this));
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr point_pub;
  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr yaw_pub;
  rclcpp::Publisher<quadrotor_msgs::msg::TakeoffLand>::SharedPtr takeoff_land_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr backcommand_pub;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr startcommand_pub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr startcommand_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr backcommand_sub;
  rclcpp::Subscription<px4_msgs::msg::InputRc>::SharedPtr rc_sub;
  rclcpp::TimerBase::SharedPtr timer;

  enum RC_EIGHT_STATE
  {
      RC_EIGHT_UP = 999,
      RC_EIGHT_MIDDLE = 1499,
      RC_EIGHT_DOWN = 1999
  };

  RC_EIGHT_STATE rc_eight_pre = RC_EIGHT_DOWN;
  bool rc_init = true;

  struct pytStr {
      double x;
      double y;
      double z;
      double yaw = -100;
      double time;
  };

  enum FLIGT_TYPE{
      PP = 1,
      PP_TIME = 2,
      PP_YAW = 3,
      PP_YAW_TIME = 4
  };

  Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;
  Eigen::Vector3d back_pos{0.0, 0.0, 1.2};
  geometry_msgs::msg::PoseStamped goal;
  vector<pytStr> pytVector;
  vector<pytStr> start_pytVector, back_pytVector;
  int counts = 0;
  double distance_ = 10.0;
  double next_distance;
  quadrotor_msgs::msg::PositionCommand cmd_yaw;
  bool time_flag = false;
  int fligt_type;
  bool trigger = false;
  int flag_start_plan;
  int flag_back_plan;

  void readpyt(string file_path)
  {
      if(file_path.empty())
      {
          RCLCPP_ERROR(this->get_logger(), "The YAML file path is empty,Failed to load pyt!");
          return;
      }
      YAML::Node pyt_yaml = YAML::LoadFile(file_path);
      YAML::Node pyt_type;
      YAML::Node pyt_type_;

      if (pyt_yaml.IsNull())
      {
          RCLCPP_ERROR(this->get_logger(), "The YAML file is empty,Failed to load pyt!");
          return;
      }

      int size_arr;
      if(fligt_type == FLIGT_TYPE::PP){
          pyt_type = pyt_yaml["test1"];
          size_arr = 3;
      }
      else if(fligt_type == FLIGT_TYPE::PP_YAW || fligt_type == FLIGT_TYPE::PP_TIME){
          if(fligt_type == FLIGT_TYPE::PP_YAW){
              pyt_type = pyt_yaml["test3"];
          }
          else{
              pyt_type = pyt_yaml["test2"];
          }
          size_arr = 4;
      }
      else{
          pyt_type = pyt_yaml["test4"];
          size_arr = 5;
      }

      pyt_type_ = pyt_yaml["test_back"];

      for (size_t i = 0; i < pyt_type.size(); i++)
      {
          if (pyt_type[i].size() != size_arr)
          {
              RCLCPP_ERROR(this->get_logger(), "Invalid point format at index %zu.", i);
              return;
          }
      }

      for (size_t i = 0; i < pyt_type_.size(); i++)
      {
          if (pyt_type_[i].size() != 3)
          {
              RCLCPP_ERROR(this->get_logger(), "Invalid back_point format at index %zu.", i);
              return;
          }
      }

      for (size_t i = 0; i < pyt_type.size(); i++)
      {
          pytStr pyt_;
          pyt_.x = pyt_type[i][0].as<double>();
          pyt_.y = pyt_type[i][1].as<double>();
          pyt_.z = pyt_type[i][2].as<double>();
          if(fligt_type == FLIGT_TYPE::PP_YAW){
              pyt_.yaw = pyt_type[i][3].as<double>();
          }
          else if(fligt_type == FLIGT_TYPE::PP_YAW_TIME){
              pyt_.yaw = pyt_type[i][3].as<double>();
              pyt_.time = pyt_type[i][4].as<double>();
              if(pyt_.time < 0){
                  RCLCPP_ERROR(this->get_logger(), "第%d个数组传入错误的等待时间", (int)i+1);
                  return;
              }
          }
          else if(fligt_type == FLIGT_TYPE::PP_TIME){
              pyt_.time = pyt_type[i][3].as<double>();
               if(pyt_.time < 0){
                  RCLCPP_ERROR(this->get_logger(), "第%d个数组传入错误的等待时间", (int)i+1);
                  return;
              }
          }
          start_pytVector.push_back(pyt_);
      }

      for (size_t i = 0; i < pyt_type_.size(); i++)
      {
          pytStr pyt_;
          pyt_.x = pyt_type_[i][0].as<double>();
          pyt_.y = pyt_type_[i][1].as<double>();
          pyt_.z = pyt_type_[i][2].as<double>();
          back_pytVector.push_back(pyt_);
      }

      RCLCPP_INFO(this->get_logger(), "Loaded pyt from YAML file:");
      for (size_t i = 0; i < start_pytVector.size(); i++)
      {
          RCLCPP_INFO(this->get_logger(), "pyt %d: [%f, %f, %f, %f, %f]", (int)i+1,
              start_pytVector[i].x, start_pytVector[i].y, start_pytVector[i].z,
              start_pytVector[i].yaw, start_pytVector[i].time);
      }
      for (size_t i = 0; i < back_pytVector.size(); i++)
      {
          RCLCPP_INFO(this->get_logger(), "back_pyt %d: [%f, %f, %f]", (int)i+1,
              back_pytVector[i].x, back_pytVector[i].y, back_pytVector[i].z);
      }
      cout << "距离判断" << next_distance <<  endl;
  }

  void odom_goal_cb(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
      odom_pos_(0) = msg->pose.pose.position.x;
      odom_pos_(1) = msg->pose.pose.position.y;
      odom_pos_(2) = msg->pose.pose.position.z;

      odom_vel_(0) = msg->twist.twist.linear.x;
      odom_vel_(1) = msg->twist.twist.linear.y;
      odom_vel_(2) = msg->twist.twist.linear.z;
  }

  void Point_send()
  {
      if(!trigger){
          return;
      }
      else if(pytVector.size() == 0)
      {
          trigger = false;
          return;
      }

      if(counts == 0){
          goal.header.stamp = this->now();
          goal.pose.position.x = pytVector[counts].x;
          goal.pose.position.y = pytVector[counts].y;
          goal.pose.position.z = pytVector[counts].z;
          point_pub->publish(goal);
          if(fligt_type == FLIGT_TYPE::PP_YAW_TIME){
              RCLCPP_INFO(this->get_logger(), "Publish the first pyt: [x:%f, y:%f, z:%f, yaw:%f, time:%f]",
                  goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
                  cmd_yaw.yaw, pytVector[counts].time);
          }
          else if(fligt_type == FLIGT_TYPE::PP_TIME){
              RCLCPP_INFO(this->get_logger(), "Publish the first pyt: [x:%f, y:%f, z:%f, time:%f]",
                  goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
                  pytVector[counts].time);
          }
          else if(fligt_type == FLIGT_TYPE::PP_YAW){
              RCLCPP_INFO(this->get_logger(), "Publish the first pyt: [x:%f, y:%f, z:%f, yaw:%f]",
                  goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
                  pytVector[counts].yaw);
          }
          else{
              RCLCPP_INFO(this->get_logger(), "Publish the first pyt: [x:%f, y:%f, z:%f]",
                  goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
          }
          counts++;
      }

      int counts_pre = counts - 1;
      Eigen::Vector3d point_cur{pytVector[counts_pre].x, pytVector[counts_pre].y, pytVector[counts_pre].z};
      distance_ = (point_cur - odom_pos_).norm();

      if (distance_ < next_distance && counts < (int)pytVector.size())
      {
          if(fligt_type == FLIGT_TYPE::PP_TIME || fligt_type == FLIGT_TYPE::PP_YAW_TIME){
              timer->cancel();
              RCLCPP_INFO(this->get_logger(), "Timer stopped.");

              double time_wait = pytVector[counts_pre].time;
              rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
                  std::chrono::duration<double>(time_wait)));

              timer->reset();
              RCLCPP_INFO(this->get_logger(), "Timer started.");
          }

          goal.header.stamp = this->now();
          goal.pose.position.x = pytVector[counts].x;
          goal.pose.position.y = pytVector[counts].y;
          goal.pose.position.z = pytVector[counts].z;
          point_pub->publish(goal);
          if(fligt_type == FLIGT_TYPE::PP_YAW_TIME){
              RCLCPP_INFO(this->get_logger(), "Publish the next pyt: [x:%f, y:%f, z:%f, yaw:%f, time:%f]",
                  goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
                  cmd_yaw.yaw, pytVector[counts].time);
          }
          else if(fligt_type == FLIGT_TYPE::PP_TIME){
              RCLCPP_INFO(this->get_logger(), "Publish the next pyt: [x:%f, y:%f, z:%f, time:%f]",
                  goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
                  pytVector[counts].time);
          }
          else if(fligt_type == FLIGT_TYPE::PP_YAW){
              RCLCPP_INFO(this->get_logger(), "Publish the next pyt: [x:%f, y:%f, z:%f, yaw:%f]",
                  goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
                  cmd_yaw.yaw);
          }
          else{
              RCLCPP_INFO(this->get_logger(), "Publish the next pyt: [x:%f, y:%f, z:%f]",
                  goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
          }
          counts++;
      }

      if (distance_ > next_distance && counts_pre < (int)pytVector.size())
      {
          if(fligt_type == FLIGT_TYPE::PP_YAW || fligt_type == FLIGT_TYPE::PP_YAW_TIME)
          {
              cmd_yaw.yaw = pytVector[counts_pre].yaw;
              yaw_pub->publish(cmd_yaw);
          }
      }
  }

  void startplan_cb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
      if(start_pytVector.size() == 0)
      {
          RCLCPP_ERROR(this->get_logger(), "No pyt loaded!");
          return;
      }

      pytVector.assign(start_pytVector.begin(), start_pytVector.end());
      counts = 0;
      trigger = true;
      RCLCPP_INFO(this->get_logger(), "Get start trigger.");
  }

  void backplan_cb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
      if(back_pytVector.size() == 0)
      {
          RCLCPP_ERROR(this->get_logger(), "No pyt loaded!");
          return;
      }

      pytVector.clear();
      pytVector.assign(back_pytVector.begin(), back_pytVector.end());
      counts = 0;
      fligt_type = FLIGT_TYPE::PP;
      trigger = true;
      RCLCPP_INFO(this->get_logger(), "Get back trigger.");
  }

  void rc_cb(px4_msgs::msg::InputRc::ConstSharedPtr pMsg)
  {
      uint16_t rc_eight_cur = pMsg->values[7];

      if (rc_eight_cur != RC_EIGHT_DOWN && rc_init){
          return;
      }
      else{
          rc_init = false;
      }

      if (rc_eight_cur == RC_EIGHT_MIDDLE && rc_eight_pre == RC_EIGHT_DOWN)
      {
          quadrotor_msgs::msg::TakeoffLand takeoff_msg;
          takeoff_msg.takeoff_land_cmd = 1;
          takeoff_land_pub->publish(takeoff_msg);
          cout << "down --> middle" << endl;
          rc_eight_pre = RC_EIGHT_MIDDLE;
          return;
      }

      if (rc_eight_cur == RC_EIGHT_UP && rc_eight_pre == RC_EIGHT_MIDDLE)
      {
          geometry_msgs::msg::PoseStamped startcommand_msg;
          startcommand_msg.header.stamp = this->now();
          startcommand_pub->publish(startcommand_msg);
          cout << "middle --> up" << endl;
          rc_eight_pre = RC_EIGHT_UP;
          return;
      }

      if (rc_eight_cur == RC_EIGHT_MIDDLE && rc_eight_pre == RC_EIGHT_UP)
      {
          geometry_msgs::msg::PoseStamped backtrigger_msg;
          backtrigger_msg.header.stamp = this->now();
          backcommand_pub->publish(backtrigger_msg);
          cout << "up --> middle" << endl;
          rc_eight_pre = RC_EIGHT_MIDDLE;
          return;
      }

      if (rc_eight_cur == RC_EIGHT_DOWN && rc_eight_pre == RC_EIGHT_MIDDLE)
      {
          quadrotor_msgs::msg::TakeoffLand takeoff_msg;
          takeoff_msg.takeoff_land_cmd = 2;
          takeoff_land_pub->publish(takeoff_msg);
          cout << "middle --> down" << endl;
          rc_eight_pre = RC_EIGHT_DOWN;
          return;
      }
  }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultipointPlanNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
   