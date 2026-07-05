#include <rclcpp/rclcpp.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <traj_utils/msg/minco_traj.hpp>
#include <optimizer/poly_traj_utils.hpp>

using namespace std;
using namespace std::chrono_literals;

class Traj2OdomNode : public rclcpp::Node
{
public:
  Traj2OdomNode() : Node("traj2odom")
  {
    double odom_hz;
    this->declare_parameter<double>("odom_hz", 100.0);
    odom_hz = this->get_parameter("odom_hz").as_double();

    one_traj_sub_ = this->create_subscription<traj_utils::msg::MINCOTraj>(
      "/broadcast_traj_to_planner", 100,
      std::bind(&Traj2OdomNode::one_traj_sub_cb, this, std::placeholders::_1));
    other_odoms_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/others_odom", 100);

    int period_ms = (int)(1000.0 / odom_hz);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(period_ms),
      std::bind(&Traj2OdomNode::timer_callback, this));
  }

private:
  struct Traj_t
  {
    poly_traj::Trajectory traj;
    bool valid;
    rclcpp::Time start_time;
    double duration;
    double last_yaw;
  };

  vector<Traj_t> trajs_;
  rclcpp::Subscription<traj_utils::msg::MINCOTraj>::SharedPtr one_traj_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr other_odoms_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  void one_traj_sub_cb(const traj_utils::msg::MINCOTraj::ConstSharedPtr &msg)
  {
    const int recv_id = msg->drone_id;

    if (msg->drone_id < 0)
    {
      RCLCPP_ERROR(this->get_logger(), "drone_id < 0 is not allowed in a swarm system!");
      return;
    }
    if (msg->order != 5)
    {
      RCLCPP_ERROR(this->get_logger(), "Only support trajectory order equals 5 now!");
      return;
    }
    if (msg->duration.size() != (msg->inner_x.size() + 1))
    {
      RCLCPP_ERROR(this->get_logger(), "WRONG trajectory parameters.");
      return;
    }
    rclcpp::Time start_time(msg->start_time);

    if ((int)trajs_.size() > recv_id &&
        (start_time - trajs_[recv_id].start_time).seconds() <= 0)
    {
      RCLCPP_WARN(this->get_logger(), "Received drone %d's trajectory out of order or duplicated, abandon it.", (int)recv_id);
      return;
    }

    rclcpp::Time t_now = this->now();
    if (abs((t_now - start_time).seconds()) > 0.25)
    {
      if (abs((t_now - start_time).seconds()) < 10.0)
      {
        RCLCPP_WARN(this->get_logger(), "Time stamp diff: Local - Remote Agent %d = %fs",
                 msg->drone_id, (t_now - start_time).seconds());
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Time stamp diff: Local - Remote Agent %d = %fs, swarm time seems not synchronized, abandon!",
                  msg->drone_id, (t_now - start_time).seconds());
        return;
      }
    }

    /* Fill up the buffer */
    if ((int)trajs_.size() <= recv_id)
    {
      for (int i = trajs_.size(); i <= recv_id; i++)
      {
        Traj_t blank;
        blank.valid = false;
        trajs_.push_back(blank);
      }
    }

    /* Store data */
    int piece_nums = msg->duration.size();
    Eigen::Matrix<double, 3, 3> headState, tailState;
    headState << msg->start_p[0], msg->start_v[0], msg->start_a[0],
        msg->start_p[1], msg->start_v[1], msg->start_a[1],
        msg->start_p[2], msg->start_v[2], msg->start_a[2];
    tailState << msg->end_p[0], msg->end_v[0], msg->end_a[0],
        msg->end_p[1], msg->end_v[1], msg->end_a[1],
        msg->end_p[2], msg->end_v[2], msg->end_a[2];
    Eigen::MatrixXd innerPts(3, piece_nums - 1);
    Eigen::VectorXd durations(piece_nums);
    for (int i = 0; i < piece_nums - 1; i++)
      innerPts.col(i) << msg->inner_x[i], msg->inner_y[i], msg->inner_z[i];
    for (int i = 0; i < piece_nums; i++)
      durations(i) = msg->duration[i];
    poly_traj::MinJerkOpt MJO;
    MJO.reset(headState, tailState, piece_nums);
    MJO.generate(innerPts, durations);

    trajs_[recv_id].traj = MJO.getTraj();
    trajs_[recv_id].start_time = start_time;
    trajs_[recv_id].valid = true;
    trajs_[recv_id].duration = trajs_[recv_id].traj.getTotalDuration();
  }

  void timer_callback()
  {
    auto t_now = this->now();

    for (int id = 0; id < (int)trajs_.size(); ++id)
    {
      if (trajs_[id].valid)
      {
        double t_to_start = (t_now - trajs_[id].start_time).seconds();
        if (t_to_start <= trajs_[id].duration)
        {
          double t = t_to_start;
          Eigen::Vector3d p = trajs_[id].traj.getPos(t);
          Eigen::Vector3d v = trajs_[id].traj.getVel(t);
          double yaw = v.head(2).norm() > 0.01 ? atan2(v(1), v(0)) : trajs_[id].last_yaw;
          trajs_[id].last_yaw = yaw;
          Eigen::AngleAxisd rotation_vector(yaw, Eigen::Vector3d::UnitZ());
          Eigen::Quaterniond q = Eigen::Quaterniond(rotation_vector);

          nav_msgs::msg::Odometry msg;
          msg.header.frame_id = string("world");
          msg.header.stamp = t_now;
          msg.child_frame_id = string("drone_") + to_string(id);
          msg.pose.pose.position.x = p(0);
          msg.pose.pose.position.y = p(1);
          msg.pose.pose.position.z = p(2);
          msg.pose.pose.orientation.w = q.w();
          msg.pose.pose.orientation.x = q.x();
          msg.pose.pose.orientation.y = q.y();
          msg.pose.pose.orientation.z = q.z();
          msg.twist.twist.linear.x = v(0);
          msg.twist.twist.linear.y = v(1);
          msg.twist.twist.linear.z = v(2);

          other_odoms_pub_->publish(msg);
        }
      }
    }
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Traj2OdomNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}