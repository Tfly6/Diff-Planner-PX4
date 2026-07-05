#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <traj_utils/msg/poly_traj.hpp>
#include <optimizer/poly_traj_utils.hpp>

using namespace Eigen;

namespace
{
rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr pos_cmd_pub;
rclcpp::Node::SharedPtr node_ptr;

quadrotor_msgs::msg::PositionCommand cmd;
bool receive_traj_ = false;
std::shared_ptr<poly_traj::Trajectory> traj_;
double traj_duration_;
rclcpp::Time start_time_;
int traj_id_;
rclcpp::Time last_diag_time_(0, 0, RCL_ROS_TIME);
bool logged_first_heartbeat_ = false;
bool logged_first_traj_ = false;
bool logged_first_cmd_ = false;
rclcpp::Time heartbeat_time_(0, 0, RCL_SYSTEM_TIME);
Eigen::Vector3d last_pos_;

double last_yaw_, last_yawdot_, slowly_flip_yaw_target_, slowly_turn_to_center_target_;
double time_forward_;
double yaw_custom_;
bool receive_yaw_ = false;
rclcpp::Time receive_yaw_time_(0, 0, RCL_SYSTEM_TIME);

void heartbeatCallback(const std_msgs::msg::Empty::SharedPtr)
{
  heartbeat_time_ = node_ptr->now();
  if (!logged_first_heartbeat_)
  {
    logged_first_heartbeat_ = true;
    RCLCPP_INFO(node_ptr->get_logger(), "[traj_server] Received first heartbeat.");
  }
}

void yawCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
{
  receive_yaw_ = true;
  receive_yaw_time_ = node_ptr->now();
  yaw_custom_ = msg->yaw;
}

void polyTrajCallback(const traj_utils::msg::PolyTraj::SharedPtr msg)
{
  if (msg->order != 5)
  {
    RCLCPP_ERROR(node_ptr->get_logger(), "[traj_server] Only support trajectory order equals 5 now!");
    return;
  }
  if (msg->duration.size() * (msg->order + 1) != msg->coef_x.size())
  {
    RCLCPP_ERROR(node_ptr->get_logger(), "[traj_server] Wrong trajectory parameters.");
    return;
  }

  int piece_nums = static_cast<int>(msg->duration.size());
  std::vector<double> dura(piece_nums);
  std::vector<poly_traj::CoefficientMat> cMats(piece_nums);
  for (int i = 0; i < piece_nums; ++i)
  {
    int i6 = i * 6;
    cMats[i].row(0) << msg->coef_x[i6 + 0], msg->coef_x[i6 + 1], msg->coef_x[i6 + 2],
        msg->coef_x[i6 + 3], msg->coef_x[i6 + 4], msg->coef_x[i6 + 5];
    cMats[i].row(1) << msg->coef_y[i6 + 0], msg->coef_y[i6 + 1], msg->coef_y[i6 + 2],
        msg->coef_y[i6 + 3], msg->coef_y[i6 + 4], msg->coef_y[i6 + 5];
    cMats[i].row(2) << msg->coef_z[i6 + 0], msg->coef_z[i6 + 1], msg->coef_z[i6 + 2],
        msg->coef_z[i6 + 3], msg->coef_z[i6 + 4], msg->coef_z[i6 + 5];
    dura[i] = msg->duration[i];
  }

  traj_ = std::make_shared<poly_traj::Trajectory>(dura, cMats);
  start_time_ = rclcpp::Time(msg->start_time);
  traj_duration_ = traj_->getTotalDuration();
  traj_id_ = msg->traj_id;
  receive_traj_ = true;
  if (!logged_first_traj_)
  {
    logged_first_traj_ = true;
    RCLCPP_INFO(
        node_ptr->get_logger(),
        "[traj_server] Received first PolyTraj: traj_id=%d, pieces=%d, duration=%.3f, start_time=%.3f",
        traj_id_, piece_nums, traj_duration_, start_time_.seconds());
  }
  else
  {
    RCLCPP_INFO(
        node_ptr->get_logger(),
        "[traj_server] Updated PolyTraj: traj_id=%d, pieces=%d, duration=%.3f, start_time=%.3f",
        traj_id_, piece_nums, traj_duration_, start_time_.seconds());
  }
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, double dt)
{
  constexpr double YAW_DOT_MAX_PER_SEC = 2 * M_PI;
  constexpr double YAW_DOT_DOT_MAX_PER_SEC = 5 * M_PI;
  std::pair<double, double> yaw_yawdot(0, 0);

  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_
                            ? traj_->getPos(t_cur + time_forward_) - pos
                            : traj_->getPos(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;
  if (receive_yaw_ && yaw_custom_ > -100.0)
  {
    if ((node_ptr->now() - receive_yaw_time_).seconds() < 0.5)
      yaw_temp = yaw_custom_;
    else
      receive_yaw_ = false;
  }

  double d_yaw = yaw_temp - last_yaw_;
  if (d_yaw >= M_PI)
    d_yaw -= 2 * M_PI;
  if (d_yaw <= -M_PI)
    d_yaw += 2 * M_PI;

  const double ydm = d_yaw >= 0 ? YAW_DOT_MAX_PER_SEC : -YAW_DOT_MAX_PER_SEC;
  const double yddm = d_yaw >= 0 ? YAW_DOT_DOT_MAX_PER_SEC : -YAW_DOT_DOT_MAX_PER_SEC;
  double d_yaw_max;
  if (fabs(last_yawdot_ + dt * yddm) <= fabs(ydm))
    d_yaw_max = last_yawdot_ * dt + 0.5 * yddm * dt * dt;
  else
  {
    double t1 = (ydm - last_yawdot_) / yddm;
    d_yaw_max = ((dt - t1) + dt) * (ydm - last_yawdot_) / 2.0;
  }

  if (fabs(d_yaw) > fabs(d_yaw_max))
    d_yaw = d_yaw_max;
  double yawdot = d_yaw / dt;
  double yaw = last_yaw_ + d_yaw;
  if (yaw > M_PI)
    yaw -= 2 * M_PI;
  if (yaw < -M_PI)
    yaw += 2 * M_PI;

  last_yaw_ = yaw;
  last_yawdot_ = yawdot;
  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yaw_temp;
  return yaw_yawdot;
}

void publish_cmd(Vector3d p, Vector3d v, Vector3d a, Vector3d j, double y, double yd)
{
  cmd.header.stamp = node_ptr->now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;
  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.jerk.x = j(0);
  cmd.jerk.y = j(1);
  cmd.jerk.z = j(2);
  cmd.yaw = y;
  cmd.yaw_dot = yd;
  pos_cmd_pub->publish(cmd);
  if (!logged_first_cmd_)
  {
    logged_first_cmd_ = true;
    RCLCPP_INFO(
        node_ptr->get_logger(),
        "[traj_server] Published first PositionCommand on /position_cmd: pos=(%.3f, %.3f, %.3f), yaw=%.3f",
        p(0), p(1), p(2), y);
  }
  last_pos_ = p;
}

void cmdCallback()
{
  if (heartbeat_time_.seconds() <= 1e-5 || !receive_traj_)
  {
    rclcpp::Time time_now = node_ptr->now();
    if ((time_now - last_diag_time_).seconds() > 2.0)
    {
      last_diag_time_ = time_now;
      RCLCPP_WARN(
          node_ptr->get_logger(),
          "[traj_server] Waiting for inputs: heartbeat=%s, trajectory=%s",
          heartbeat_time_.seconds() > 1e-5 ? "yes" : "no",
          receive_traj_ ? "yes" : "no");
    }
    return;
  }

  rclcpp::Time time_now = node_ptr->now();
  if ((time_now - heartbeat_time_).seconds() > 0.5)
  {
    RCLCPP_ERROR(node_ptr->get_logger(), "[traj_server] Lost heartbeat from the planner, is it dead?");
    receive_traj_ = false;
    publish_cmd(last_pos_, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_yaw_, 0);
  }

  double t_cur = (time_now - start_time_).seconds();
  Eigen::Vector3d pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d vel = Eigen::Vector3d::Zero();
  Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  Eigen::Vector3d jer = Eigen::Vector3d::Zero();
  static rclcpp::Time time_last = node_ptr->now();

  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    pos = traj_->getPos(t_cur);
    vel = traj_->getVel(t_cur);
    acc = traj_->getAcc(t_cur);
    jer = traj_->getJer(t_cur);
    auto yaw_yawdot = calculate_yaw(t_cur, pos, 0.01);
    time_last = time_now;
    last_yaw_ = yaw_yawdot.first;
    last_pos_ = pos;

    slowly_flip_yaw_target_ = yaw_yawdot.first + M_PI;
    if (slowly_flip_yaw_target_ > M_PI)
      slowly_flip_yaw_target_ -= 2 * M_PI;
    if (slowly_flip_yaw_target_ < -M_PI)
      slowly_flip_yaw_target_ += 2 * M_PI;
    constexpr double CENTER[2] = {0.0, 0.0};
    slowly_turn_to_center_target_ = atan2(CENTER[1] - pos(1), CENTER[0] - pos(0));
    publish_cmd(pos, vel, acc, jer, yaw_yawdot.first, yaw_yawdot.second);
  }
  else if (t_cur >= traj_duration_)
  {
    pos = traj_->getPos(traj_duration_);
    publish_cmd(pos, Vector3d::Zero(), Vector3d::Zero(), Vector3d::Zero(), last_yaw_, 0.0);
    time_last = time_now;
  }
}
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  node_ptr = std::make_shared<rclcpp::Node>("traj_server");

  auto poly_traj_sub = node_ptr->create_subscription<traj_utils::msg::PolyTraj>("planning/trajectory", 10, polyTrajCallback);
  auto yaw_sub = node_ptr->create_subscription<quadrotor_msgs::msg::PositionCommand>("/planning/yaw", 10, yawCallback);
  auto heartbeat_sub = node_ptr->create_subscription<std_msgs::msg::Empty>("heartbeat", 10, heartbeatCallback);

  pos_cmd_pub = node_ptr->create_publisher<quadrotor_msgs::msg::PositionCommand>("/position_cmd", 50);

  node_ptr->declare_parameter<double>("traj_server.time_forward", -1.0);
  node_ptr->get_parameter("traj_server.time_forward", time_forward_);
  last_yaw_ = 0.0;
  last_yawdot_ = 0.0;

  auto timer = node_ptr->create_wall_timer(std::chrono::milliseconds(10), cmdCallback);
  (void)poly_traj_sub;
  (void)yaw_sub;
  (void)heartbeat_sub;
  (void)timer;

  rclcpp::sleep_for(std::chrono::seconds(1));
  RCLCPP_INFO(node_ptr->get_logger(), "[traj_server] ready.");
  rclcpp::spin(node_ptr);
  rclcpp::shutdown();
  return 0;
}
