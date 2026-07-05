#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <functional>
#include <quadrotor_msgs/msg/goal_set.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <uav_utils/geometry_utils.h>

using namespace std;
using namespace std::chrono_literals;

class RandomGoalsNode : public rclcpp::Node
{
public:
  RandomGoalsNode() : Node("random_goals")
  {
    srand(floor(this->now().seconds() * 10));

    this->declare_parameter<int>("drone_num", -1);
    this->declare_parameter<int>("goal_num", -1);

    int drone_num = this->get_parameter("drone_num").as_int();
    int goal_num = this->get_parameter("goal_num").as_int();

    goals_.resize(goal_num);
    for (int i = 0; i < goal_num; ++i)
    {
      this->declare_parameter<vector<double>>("goal" + to_string(i), vector<double>());
      vector<double> pt = this->get_parameter("goal" + to_string(i)).as_double_array();
      goals_[i].p << pt[0], pt[1], pt[2];
      goals_[i].occupied = false;
    }

    drones_.resize(drone_num);
    for (int i = 0; i < drone_num; ++i)
    {
      auto cb = [this, i](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        this->odoms_sim_sub_cb(msg, i);
      };
      odoms_sim_sub_.push_back(
        this->create_subscription<nav_msgs::msg::Odometry>(
          "/drone_" + to_string(i) + "_visual_slam/odom", 1000, cb));
    }

    combined_odoms_sim_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/others_odom", 1000,
      std::bind(&RandomGoalsNode::combined_odoms_sim_sub_cb, this, std::placeholders::_1));

    goals_pub_ = this->create_publisher<quadrotor_msgs::msg::GoalSet>("/goal_user2brig", 10);

    count_.resize(goal_num);
    fill(count_.begin(), count_.end(), 0);

    timer_ = this->create_wall_timer(
      100ms, std::bind(&RandomGoalsNode::timer_callback, this));
  }

private:
  struct Drone_Info_t
  {
    Eigen::Vector3d goal;
    int goal_id{-1};
    Eigen::Vector3d cur_p;
    double cur_yaw;
    Eigen::Vector3d last_p;
    rclcpp::Time arrived_time;
    bool arrived_for_a_while{true};
    bool odom_received{false};
  };
  struct Goal_t
  {
    Eigen::Vector3d p;
    bool occupied;
  };

  std::vector<Drone_Info_t> drones_;
  std::vector<Goal_t> goals_;
  std::vector<int> count_;

  rclcpp::Publisher<quadrotor_msgs::msg::GoalSet>::SharedPtr goals_pub_;
  std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> odoms_sim_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr combined_odoms_sim_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void set_odom_data(const nav_msgs::msg::Odometry::ConstSharedPtr &msg, const int &drone_id)
  {
    drones_[drone_id].odom_received = true;
    drones_[drone_id].last_p = drones_[drone_id].cur_p;
    drones_[drone_id].cur_p << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z;
    drones_[drone_id].cur_yaw = uav_utils::get_yaw_from_quaternion(Eigen::Quaterniond(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z));
  }

  void odoms_sim_sub_cb(const nav_msgs::msg::Odometry::ConstSharedPtr &msg, const int &drone_id)
  {
    set_odom_data(msg, drone_id);
  }

  void combined_odoms_sim_sub_cb(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    int id = atoi(msg->child_frame_id.substr(6, 10).c_str());
    if (msg->child_frame_id.substr(0, 6) != string("drone_") || id >= (int)drones_.size())
    {
      RCLCPP_ERROR(this->get_logger(),
        "[random_goals_node] Wrong child_frame_id: %s, or wrong drone_id: %d",
        msg->child_frame_id.substr(0, 6).c_str(), id);
      return;
    }
    set_odom_data(msg, id);
  }

  void timer_callback()
  {
    rclcpp::Time t_now = this->now();
    int drone_num = drones_.size();
    int goal_num = goals_.size();

    for (int i = 0; i < drone_num; ++i)
    {
      double d_to_goal = (drones_[i].cur_p - drones_[i].goal).norm();
      double last_d_to_goal = (drones_[i].last_p - drones_[i].goal).norm();
      if (d_to_goal > 0.1 || last_d_to_goal > 0.1)
        drones_[i].arrived_time = t_now;
      drones_[i].arrived_for_a_while |= ((t_now - drones_[i].arrived_time).seconds() > 2);
    }

    int drone_trials = 0;
    while (drone_trials < drone_num)
    {
      int d_id = floor(((double)rand() / RAND_MAX) * drone_num);
      if (drones_[d_id].odom_received && drones_[d_id].arrived_for_a_while)
      {
        int goal_trials = 0;
        while (goal_trials < goal_num)
        {
          int g_id = floor(((double)rand() / RAND_MAX) * goal_num);
          double ang = acos(((goals_[g_id].p - drones_[d_id].cur_p).normalized()).dot(
            (Eigen::Vector3d(cos(drones_[d_id].cur_yaw), sin(drones_[d_id].cur_yaw), 0)).normalized()));
          if (!goals_[g_id].occupied && ang > 0 && ang < M_PI / 6)
          {
            if (drones_[d_id].goal_id >= 0)
              goals_[drones_[d_id].goal_id].occupied = false;
            goals_[g_id].occupied = true;
            drones_[d_id].arrived_for_a_while = false;
            drones_[d_id].goal = goals_[g_id].p;
            drones_[d_id].goal_id = g_id;
            quadrotor_msgs::msg::GoalSet msg;
            msg.drone_id = d_id;
            msg.goal[0] = drones_[d_id].goal(0);
            msg.goal[1] = drones_[d_id].goal(1);
            msg.goal[2] = drones_[d_id].goal(2);
            goals_pub_->publish(msg);
            cout << "drone_id=" << d_id << " goal=" << drones_[d_id].goal.transpose() << endl;
            return;
          }

          goal_trials++;
        }
      }

      drone_trials++;
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RandomGoalsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
