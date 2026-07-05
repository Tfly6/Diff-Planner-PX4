#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <iostream>
#include <quadrotor_msgs/msg/goal_set.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <uav_utils/geometry_utils.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std;
using namespace std::chrono_literals;

class AssignGoalsNode : public rclcpp::Node
{
public:
  AssignGoalsNode() : Node("assign_goals")
  {
    srand(floor(this->now().seconds() * 10));

    selected_drones_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/rviz_selected_drones", 100,
      std::bind(&AssignGoalsNode::selected_drones_cb, this, std::placeholders::_1));
    user_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal", 10,
      std::bind(&AssignGoalsNode::user_goal_cb, this, std::placeholders::_1));

    goals_pub_ = this->create_publisher<quadrotor_msgs::msg::GoalSet>("/goal_user2brig", 10);
    new_goals_arrow_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/new_goals_arrow", 10);

    RCLCPP_INFO(this->get_logger(), "[assign_goals_node]Start running.");

    timer_ = this->create_wall_timer(
      10ms, std::bind(&AssignGoalsNode::timer_callback, this));
  }

private:
  struct Selected_t
  {
    int drone_id;
    Eigen::Vector3d p;
  };
  vector<Selected_t> drones_;

  rclcpp::Publisher<quadrotor_msgs::msg::GoalSet>::SharedPtr goals_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr new_goals_arrow_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr selected_drones_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr user_goal_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Time last_publish_time_;
  bool need_clear_ = false;

  void displayArrowList(const vector<Eigen::Vector3d> &start, const vector<Eigen::Vector3d> &end,
                        const double scale, const int id, const int32_t action)
  {
    if (start.size() != end.size())
    {
      RCLCPP_ERROR(this->get_logger(), "start.size() != end.size(), return");
      return;
    }

    visualization_msgs::msg::MarkerArray array;

    visualization_msgs::msg::Marker arrow;
    arrow.header.frame_id = "world";
    arrow.header.stamp = this->now();
    arrow.type = visualization_msgs::msg::Marker::ARROW;
    arrow.action = action;

    arrow.color.r = 0;
    arrow.color.g = 0;
    arrow.color.b = 0;
    arrow.color.a = 1.0;
    arrow.scale.x = scale;
    arrow.scale.y = 4 * scale;
    arrow.scale.z = 4 * scale;

    for (int i = 0; i < int(start.size()); i++)
    {
      geometry_msgs::msg::Point st, ed;
      st.x = start[i](0);
      st.y = start[i](1);
      st.z = start[i](2);
      ed.x = end[i](0);
      ed.y = end[i](1);
      ed.z = end[i](2);
      arrow.points.clear();
      arrow.points.push_back(st);
      arrow.points.push_back(ed);
      arrow.id = i + id;

      array.markers.push_back(arrow);
    }

    new_goals_arrow_pub_->publish(array);
  }

  void selected_drones_cb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
    static rclcpp::Time last_select_time = rclcpp::Time(0, 0, RCL_ROS_TIME);
    rclcpp::Time t_now = this->now();
    if ((t_now - last_select_time).seconds() > 2)
    {
      drones_.clear();
    }
    Selected_t drone;
    drone.drone_id = atoi(msg->header.frame_id.substr(6, 10).c_str());
    drone.p << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    drones_.push_back(drone);

    cout.precision(3);
    cout << "received drone " << drone.drone_id << " at " << drone.p.transpose()
         << ", total:" << drones_.size() << endl;

    last_select_time = t_now;
  }

  void user_goal_cb(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
    Eigen::Vector3d center = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < drones_.size(); ++i)
    {
      center += drones_[i].p;
    }
    center /= drones_.size();

    Eigen::Vector3d user_goal(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    Eigen::Vector3d movment = user_goal - center;

    vector<Eigen::Vector3d> each_one_starts(drones_.size()), each_one_goals(drones_.size());
    for (size_t i = 0; i < drones_.size(); ++i)
    {
      each_one_starts[i] = drones_[i].p;
      each_one_goals[i] = drones_[i].p + movment;
      cout.precision(3);
      cout << "drone " << drones_[i].drone_id
           << ", start=" << drones_[i].p.transpose()
           << ", end=" << each_one_goals[i].transpose() << endl;
    }

    displayArrowList(each_one_starts, each_one_goals, 0.05, 0,
                     visualization_msgs::msg::Marker::ADD);
    last_publish_time_ = this->now();
    need_clear_ = true;

    for (size_t i = 0; i < drones_.size(); ++i)
    {
      quadrotor_msgs::msg::GoalSet goal_msg;
      goal_msg.drone_id = drones_[i].drone_id;
      goal_msg.goal[0] = each_one_goals[i](0);
      goal_msg.goal[1] = each_one_goals[i](1);
      goal_msg.goal[2] = each_one_goals[i](2);
      goals_pub_->publish(goal_msg);
      rclcpp::sleep_for(10ms);
    }
  }

  void timer_callback()
  {
    if (need_clear_ && (this->now() - last_publish_time_).seconds() > 2)
    {
      need_clear_ = false;
      std::vector<Eigen::Vector3d> blank(1);
      blank[0] = Eigen::Vector3d::Zero();
      displayArrowList(blank, blank, 0.05, 0,
                       visualization_msgs::msg::Marker::DELETEALL);
      cout << "DELETEALL Arrows." << endl;
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AssignGoalsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
