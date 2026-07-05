#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include "armadillo"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pose_utils.h"
#include "quadrotor_msgs/msg/position_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"

using arma::colvec;
using arma::det;
using arma::eig_sym;
using arma::mat;
using arma::norm;
using arma::trans;
using arma::zeros;
using std::string;

class OdomVisualizationNode : public rclcpp::Node
{
public:
  OdomVisualizationNode()
  : Node("odom_visualization")
  {
    mesh_resource_ = declare_parameter<string>(
      "mesh_resource", "package://odom_visualization/meshes/fake_drone.dae");
    color_r_ = declare_parameter<double>("color.r", 1.0);
    color_g_ = declare_parameter<double>("color.g", 0.0);
    color_b_ = declare_parameter<double>("color.b", 0.0);
    color_a_ = declare_parameter<double>("color.a", 1.0);
    origin_ = declare_parameter<bool>("origin", false);
    scale_ = declare_parameter<double>("robot_scale", 2.0);
    frame_id_ = declare_parameter<string>("frame_id", "world");
    rotate_yaw_ = declare_parameter<double>("rotate_yaw_deg", 0.0);
    cross_config_ = declare_parameter<bool>("cross_config", false);
    tf45_ = declare_parameter<bool>("tf45", false);
    cov_scale_ = declare_parameter<double>("covariance_scale", 100.0);
    cov_pos_ = declare_parameter<bool>("covariance_position", false);
    cov_vel_ = declare_parameter<bool>("covariance_velocity", false);
    cov_color_ = declare_parameter<bool>("covariance_color", false);
    drone_id_ = declare_parameter<int>("drone_id", -1);

    const auto latched_qos = rclcpp::QoS(rclcpp::KeepLast(100)).reliable().transient_local();
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("pose", latched_qos);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("path", latched_qos);
    vel_pub_ = create_publisher<visualization_msgs::msg::Marker>("velocity", latched_qos);
    cov_pub_ = create_publisher<visualization_msgs::msg::Marker>("covariance", latched_qos);
    cov_vel_pub_ = create_publisher<visualization_msgs::msg::Marker>("covariance_velocity", latched_qos);
    traj_pub_ = create_publisher<visualization_msgs::msg::Marker>("trajectory", latched_qos);
    sensor_pub_ = create_publisher<visualization_msgs::msg::Marker>("sensor", latched_qos);
    mesh_pub_ = create_publisher<visualization_msgs::msg::Marker>("robot", latched_qos);
    height_pub_ = create_publisher<sensor_msgs::msg::Range>("height", latched_qos);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odom", rclcpp::QoS(100),
      std::bind(&OdomVisualizationNode::odomCallback, this, std::placeholders::_1));
    cmd_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
      "cmd", rclcpp::QoS(100),
      std::bind(&OdomVisualizationNode::cmdCallback, this, std::placeholders::_1));
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (msg->header.frame_id == "null") {
      return;
    }

    colvec pose(6);
    pose(0) = msg->pose.pose.position.x;
    pose(1) = msg->pose.pose.position.y;
    pose(2) = msg->pose.pose.position.z;

    colvec q(4);
    q(0) = msg->pose.pose.orientation.w;
    q(1) = msg->pose.pose.orientation.x;
    q(2) = msg->pose.pose.orientation.y;
    q(3) = msg->pose.pose.orientation.z;
    pose.rows(3, 5) = R_to_ypr(quaternion_to_R(q));

    colvec vel(3);
    vel(0) = msg->twist.twist.linear.x;
    vel(1) = msg->twist.twist.linear.y;
    vel(2) = msg->twist.twist.linear.z;

    if (origin_ && !is_origin_set_) {
      is_origin_set_ = true;
      pose_origin_ = pose;
    }
    if (origin_) {
      vel = trans(ypr_to_R(pose.rows(3, 5))) * vel;
      pose = pose_update(pose_inverse(pose_origin_), pose);
      vel = ypr_to_R(pose.rows(3, 5)) * vel;
    }

    publishPose(msg, pose);
    const auto velocity_quat = publishVelocity(msg, pose, vel);
    publishPath(msg);
    const auto [r, g, b] = covarianceColor(msg);
    publishPositionCovariance(msg, pose, r, g, b);
    publishVelocityCovariance(msg, pose, r, g, b);
    publishTrajectory(msg, pose, r, g, b);
    publishSensorText(msg, pose, velocity_quat);
    publishHeight(msg);
    publishMesh(msg);
    publishTf45(msg, pose);
  }

  void cmdCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr cmd)
  {
    if (cmd->header.frame_id == "null") {
      return;
    }

    visualization_msgs::msg::Marker mesh_ros;
    mesh_ros.header.frame_id = frame_id_;
    mesh_ros.header.stamp = cmd->header.stamp;
    mesh_ros.ns = "drone";
    mesh_ros.id = drone_id_;
    mesh_ros.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    mesh_ros.action = visualization_msgs::msg::Marker::ADD;
    mesh_ros.pose.position.x = cmd->position.x;
    mesh_ros.pose.position.y = cmd->position.y;
    mesh_ros.pose.position.z = cmd->position.z;

    colvec q(4);
    q(0) = 1.0;
    q(1) = 0.0;
    q(2) = 0.0;
    q(3) = 0.0;
    if (cross_config_) {
      colvec ypr = R_to_ypr(quaternion_to_R(q));
      ypr(0) += 45.0 * PI / 180.0;
      q = R_to_quaternion(ypr_to_R(ypr));
    }

    mesh_ros.pose.orientation.w = q(0);
    mesh_ros.pose.orientation.x = q(1);
    mesh_ros.pose.orientation.y = q(2);
    mesh_ros.pose.orientation.z = q(3);
    mesh_ros.scale.x = 2.0;
    mesh_ros.scale.y = 2.0;
    mesh_ros.scale.z = 2.0;
    mesh_ros.color.a = color_a_;
    mesh_ros.color.r = color_r_;
    mesh_ros.color.g = color_g_;
    mesh_ros.color.b = color_b_;
    mesh_ros.mesh_resource = mesh_resource_;
    mesh_pub_->publish(mesh_ros);
  }

  void publishPose(const nav_msgs::msg::Odometry::SharedPtr & msg, const colvec & pose)
  {
    geometry_msgs::msg::PoseStamped pose_ros;
    pose_ros.header = msg->header;
    pose_ros.header.frame_id = "world";
    pose_ros.pose.position.x = pose(0);
    pose_ros.pose.position.y = pose(1);
    pose_ros.pose.position.z = pose(2);

    colvec q = R_to_quaternion(ypr_to_R(pose.rows(3, 5)));
    pose_ros.pose.orientation.w = q(0);
    pose_ros.pose.orientation.x = q(1);
    pose_ros.pose.orientation.y = q(2);
    pose_ros.pose.orientation.z = q(3);

    last_pose_ros_ = pose_ros;
    pose_pub_->publish(pose_ros);
  }

  colvec publishVelocity(const nav_msgs::msg::Odometry::SharedPtr & msg, const colvec & pose, const colvec & vel)
  {
    colvec ypr_vel(3);
    ypr_vel(0) = std::atan2(vel(1), vel(0));
    ypr_vel(1) = -std::atan2(vel(2), norm(vel.rows(0, 1), 2));
    ypr_vel(2) = 0.0;

    colvec q = R_to_quaternion(ypr_to_R(ypr_vel));

    visualization_msgs::msg::Marker vel_ros;
    vel_ros.header.frame_id = "world";
    vel_ros.header.stamp = msg->header.stamp;
    vel_ros.ns = "velocity";
    vel_ros.id = drone_id_;
    vel_ros.type = visualization_msgs::msg::Marker::ARROW;
    vel_ros.action = visualization_msgs::msg::Marker::ADD;
    vel_ros.pose.position.x = pose(0);
    vel_ros.pose.position.y = pose(1);
    vel_ros.pose.position.z = pose(2);
    vel_ros.pose.orientation.w = q(0);
    vel_ros.pose.orientation.x = q(1);
    vel_ros.pose.orientation.y = q(2);
    vel_ros.pose.orientation.z = q(3);
    vel_ros.scale.x = norm(vel, 2);
    vel_ros.scale.y = 0.05;
    vel_ros.scale.z = 0.05;
    vel_ros.color.a = 1.0;
    vel_ros.color.r = color_r_;
    vel_ros.color.g = color_g_;
    vel_ros.color.b = color_b_;
    vel_pub_->publish(vel_ros);

    return q;
  }

  void publishPath(const nav_msgs::msg::Odometry::SharedPtr & msg)
  {
    rclcpp::Time current(msg->header.stamp);
    if (prev_path_time_.nanoseconds() == 0 || (current - prev_path_time_).seconds() > 0.1) {
      prev_path_time_ = current;
      path_ros_.header = last_pose_ros_.header;
      path_ros_.poses.push_back(last_pose_ros_);
      path_pub_->publish(path_ros_);
    }
  }

  std::tuple<double, double, double> covarianceColor(const nav_msgs::msg::Odometry::SharedPtr & msg) const
  {
    double r = 1.0;
    double g = 1.0;
    double b = 1.0;
    const bool gps = static_cast<bool>(msg->twist.covariance[33]);
    const bool vision = static_cast<bool>(msg->twist.covariance[34]);
    const bool laser = static_cast<bool>(msg->twist.covariance[35]);
    if (cov_color_) {
      r = gps;
      g = vision;
      b = laser;
    }
    return {r, g, b};
  }

  void publishPositionCovariance(
    const nav_msgs::msg::Odometry::SharedPtr & msg, const colvec & pose, double r, double g, double b)
  {
    if (!cov_pos_) {
      return;
    }

    mat p(6, 6);
    for (int j = 0; j < 6; ++j) {
      for (int i = 0; i < 6; ++i) {
        p(i, j) = msg->pose.covariance[i + j * 6];
      }
    }

    colvec eig_val;
    mat eig_vec;
    eig_sym(eig_val, eig_vec, p.submat(0, 0, 2, 2));
    makeRightHanded(eig_vec);

    visualization_msgs::msg::Marker cov_ros;
    cov_ros.header.frame_id = "world";
    cov_ros.header.stamp = msg->header.stamp;
    cov_ros.ns = "covariance";
    cov_ros.id = drone_id_;
    cov_ros.type = visualization_msgs::msg::Marker::SPHERE;
    cov_ros.action = visualization_msgs::msg::Marker::ADD;
    cov_ros.pose.position.x = pose(0);
    cov_ros.pose.position.y = pose(1);
    cov_ros.pose.position.z = pose(2);

    const colvec q = R_to_quaternion(eig_vec);
    cov_ros.pose.orientation.w = q(0);
    cov_ros.pose.orientation.x = q(1);
    cov_ros.pose.orientation.y = q(2);
    cov_ros.pose.orientation.z = q(3);
    cov_ros.scale.x = std::sqrt(eig_val(0)) * cov_scale_;
    cov_ros.scale.y = std::sqrt(eig_val(1)) * cov_scale_;
    cov_ros.scale.z = std::sqrt(eig_val(2)) * cov_scale_;
    cov_ros.color.a = 0.4;
    cov_ros.color.r = r * 0.5;
    cov_ros.color.g = g * 0.5;
    cov_ros.color.b = b * 0.5;
    cov_pub_->publish(cov_ros);
  }

  void publishVelocityCovariance(
    const nav_msgs::msg::Odometry::SharedPtr & msg, const colvec & pose, double r, double g, double b)
  {
    if (!cov_vel_) {
      return;
    }

    mat p(3, 3);
    for (int j = 0; j < 3; ++j) {
      for (int i = 0; i < 3; ++i) {
        p(i, j) = msg->twist.covariance[i + j * 6];
      }
    }
    const mat rot = ypr_to_R(pose.rows(3, 5));
    p = rot * p * trans(rot);

    colvec eig_val;
    mat eig_vec;
    eig_sym(eig_val, eig_vec, p);
    makeRightHanded(eig_vec);

    visualization_msgs::msg::Marker cov_vel_ros;
    cov_vel_ros.header.frame_id = "world";
    cov_vel_ros.header.stamp = msg->header.stamp;
    cov_vel_ros.ns = "covariance_velocity";
    cov_vel_ros.id = drone_id_;
    cov_vel_ros.type = visualization_msgs::msg::Marker::SPHERE;
    cov_vel_ros.action = visualization_msgs::msg::Marker::ADD;
    cov_vel_ros.pose.position.x = pose(0);
    cov_vel_ros.pose.position.y = pose(1);
    cov_vel_ros.pose.position.z = pose(2);

    const colvec q = R_to_quaternion(eig_vec);
    cov_vel_ros.pose.orientation.w = q(0);
    cov_vel_ros.pose.orientation.x = q(1);
    cov_vel_ros.pose.orientation.y = q(2);
    cov_vel_ros.pose.orientation.z = q(3);
    cov_vel_ros.scale.x = std::sqrt(eig_val(0)) * cov_scale_;
    cov_vel_ros.scale.y = std::sqrt(eig_val(1)) * cov_scale_;
    cov_vel_ros.scale.z = std::sqrt(eig_val(2)) * cov_scale_;
    cov_vel_ros.color.a = 0.4;
    cov_vel_ros.color.r = r;
    cov_vel_ros.color.g = g;
    cov_vel_ros.color.b = b;
    cov_vel_pub_->publish(cov_vel_ros);
  }

  void publishTrajectory(
    const nav_msgs::msg::Odometry::SharedPtr & msg, const colvec & pose, double r, double g, double b)
  {
    rclcpp::Time current(msg->header.stamp);
    if (!has_prev_traj_pose_) {
      prev_traj_pose_ = pose;
      prev_traj_time_ = current;
      has_prev_traj_pose_ = true;
      return;
    }
    if ((current - prev_traj_time_).seconds() <= 0.5) {
      return;
    }

    visualization_msgs::msg::Marker traj_ros;
    traj_ros.header.frame_id = "world";
    traj_ros.header.stamp = msg->header.stamp;
    traj_ros.ns = "trajectory";
    traj_ros.type = visualization_msgs::msg::Marker::LINE_LIST;
    traj_ros.action = visualization_msgs::msg::Marker::ADD;
    traj_ros.pose.orientation.w = 1.0;
    traj_ros.scale.x = 0.1;
    traj_ros.color.g = 1.0;
    traj_ros.color.a = 0.8;

    geometry_msgs::msg::Point p0;
    p0.x = prev_traj_pose_(0);
    p0.y = prev_traj_pose_(1);
    p0.z = prev_traj_pose_(2);
    geometry_msgs::msg::Point p1;
    p1.x = pose(0);
    p1.y = pose(1);
    p1.z = pose(2);
    traj_ros.points.push_back(p0);
    traj_ros.points.push_back(p1);

    std_msgs::msg::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = 1.0;
    traj_ros.colors.push_back(color);
    traj_ros.colors.push_back(color);
    traj_pub_->publish(traj_ros);

    prev_traj_pose_ = pose;
    prev_traj_time_ = current;
  }

  void publishSensorText(
    const nav_msgs::msg::Odometry::SharedPtr & msg, const colvec & pose, const colvec & q)
  {
    const bool gps = static_cast<bool>(msg->twist.covariance[33]);
    const bool vision = static_cast<bool>(msg->twist.covariance[34]);
    const bool laser = static_cast<bool>(msg->twist.covariance[35]);

    visualization_msgs::msg::Marker sensor_ros;
    sensor_ros.header.frame_id = "world";
    sensor_ros.header.stamp = msg->header.stamp;
    sensor_ros.ns = "sensor";
    sensor_ros.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    sensor_ros.action = visualization_msgs::msg::Marker::ADD;
    sensor_ros.pose.position.x = pose(0);
    sensor_ros.pose.position.y = pose(1);
    sensor_ros.pose.position.z = pose(2) + 1.0;
    sensor_ros.pose.orientation.w = q(0);
    sensor_ros.pose.orientation.x = q(1);
    sensor_ros.pose.orientation.y = q(2);
    sensor_ros.pose.orientation.z = q(3);
    sensor_ros.text = "| " + string(gps ? " GPS " : "") + string(vision ? " Vision " : "") +
      string(laser ? " Laser " : "") + " |";
    sensor_ros.color.a = 1.0;
    sensor_ros.color.r = 1.0;
    sensor_ros.color.g = 1.0;
    sensor_ros.color.b = 1.0;
    sensor_ros.scale.z = 0.5;
    sensor_pub_->publish(sensor_ros);
  }

  void publishHeight(const nav_msgs::msg::Odometry::SharedPtr & msg)
  {
    sensor_msgs::msg::Range height_ros;
    height_ros.header.frame_id = "height";
    height_ros.header.stamp = msg->header.stamp;
    height_ros.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;
    height_ros.field_of_view = 5.0 * M_PI / 180.0;
    height_ros.min_range = -100.0;
    height_ros.max_range = 100.0;
    height_ros.range = msg->twist.covariance[32];
    height_pub_->publish(height_ros);
  }

  void publishMesh(const nav_msgs::msg::Odometry::SharedPtr & msg)
  {
    visualization_msgs::msg::Marker mesh_ros;
    mesh_ros.header.frame_id = frame_id_;
    mesh_ros.header.stamp = msg->header.stamp;
    mesh_ros.ns = "drone";
    mesh_ros.id = drone_id_;
    mesh_ros.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
    mesh_ros.action = visualization_msgs::msg::Marker::ADD;
    mesh_ros.pose.position.x = msg->pose.pose.position.x;
    mesh_ros.pose.position.y = msg->pose.pose.position.y;
    mesh_ros.pose.position.z = msg->pose.pose.position.z;

    colvec q(4);
    q(0) = msg->pose.pose.orientation.w;
    q(1) = msg->pose.pose.orientation.x;
    q(2) = msg->pose.pose.orientation.y;
    q(3) = msg->pose.pose.orientation.z;
    colvec ypr = R_to_ypr(quaternion_to_R(q));
    ypr(0) += rotate_yaw_ * PI / 180.0;
    q = R_to_quaternion(ypr_to_R(ypr));
    if (cross_config_) {
      colvec ypr_cross = R_to_ypr(quaternion_to_R(q));
      ypr_cross(0) += 45.0 * PI / 180.0;
      q = R_to_quaternion(ypr_to_R(ypr_cross));
    }

    mesh_ros.pose.orientation.w = q(0);
    mesh_ros.pose.orientation.x = q(1);
    mesh_ros.pose.orientation.y = q(2);
    mesh_ros.pose.orientation.z = q(3);
    mesh_ros.scale.x = scale_;
    mesh_ros.scale.y = scale_;
    mesh_ros.scale.z = scale_;
    mesh_ros.color.a = color_a_;
    mesh_ros.color.r = color_r_;
    mesh_ros.color.g = color_g_;
    mesh_ros.color.b = color_b_;
    mesh_ros.mesh_resource = mesh_resource_;
    mesh_pub_->publish(mesh_ros);
  }

  void publishTf45(const nav_msgs::msg::Odometry::SharedPtr & msg, const colvec & pose)
  {
    if (!tf45_) {
      return;
    }

    colvec q(4);
    q(0) = msg->pose.pose.orientation.w;
    q(1) = msg->pose.pose.orientation.x;
    q(2) = msg->pose.pose.orientation.y;
    q(3) = msg->pose.pose.orientation.z;

    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = msg->header.stamp;
    transform.header.frame_id = "world";
    transform.child_frame_id = drone_id_ == -1 ? "base" : "base" + std::to_string(drone_id_);
    transform.transform.translation.x = pose(0);
    transform.transform.translation.y = pose(1);
    transform.transform.translation.z = pose(2);
    transform.transform.rotation.w = q(0);
    transform.transform.rotation.x = q(1);
    transform.transform.rotation.y = q(2);
    transform.transform.rotation.z = q(3);
    tf_broadcaster_->sendTransform(transform);

    sendFixedChildTransform(
      msg->header.stamp, transform.child_frame_id,
      drone_id_ == -1 ? "laser" : "laser" + std::to_string(drone_id_),
      yawQuaternion(45.0 * PI / 180.0));
    sendFixedChildTransform(
      msg->header.stamp, transform.child_frame_id,
      drone_id_ == -1 ? "vision" : "vision" + std::to_string(drone_id_),
      yawQuaternion(45.0 * PI / 180.0));

    tf2::Quaternion q90;
    q90.setRPY(0.0, 90.0 * PI / 180.0, 0.0);
    sendFixedChildTransform(
      msg->header.stamp, transform.child_frame_id,
      drone_id_ == -1 ? "height" : "height" + std::to_string(drone_id_),
      q90);
  }

  tf2::Quaternion yawQuaternion(double yaw) const
  {
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);
    return q;
  }

  void sendFixedChildTransform(
    const builtin_interfaces::msg::Time & stamp, const string & parent, const string & child,
    const tf2::Quaternion & rotation)
  {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = stamp;
    transform.header.frame_id = parent;
    transform.child_frame_id = child;
    transform.transform.rotation.w = rotation.w();
    transform.transform.rotation.x = rotation.x();
    transform.transform.rotation.y = rotation.y();
    transform.transform.rotation.z = rotation.z();
    tf_broadcaster_->sendTransform(transform);
  }

  void makeRightHanded(mat & eig_vec) const
  {
    if (det(eig_vec) >= 0.0) {
      return;
    }
    for (int k = 0; k < 3; ++k) {
      mat eig_vec_rev = eig_vec;
      eig_vec_rev.col(k) *= -1.0;
      if (det(eig_vec_rev) > 0.0) {
        eig_vec = eig_vec_rev;
        return;
      }
    }
  }

  string mesh_resource_;
  string frame_id_;
  double color_r_{1.0};
  double color_g_{0.0};
  double color_b_{0.0};
  double color_a_{1.0};
  double cov_scale_{100.0};
  double scale_{2.0};
  double rotate_yaw_{0.0};
  bool cross_config_{false};
  bool tf45_{false};
  bool cov_pos_{false};
  bool cov_vel_{false};
  bool cov_color_{false};
  bool origin_{false};
  bool is_origin_set_{false};
  int drone_id_{-1};

  colvec pose_origin_{zeros<colvec>(6)};
  colvec prev_traj_pose_{zeros<colvec>(6)};
  bool has_prev_traj_pose_{false};

  geometry_msgs::msg::PoseStamped last_pose_ros_;
  nav_msgs::msg::Path path_ros_;
  rclcpp::Time prev_path_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time prev_traj_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cov_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr cov_vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr sensor_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr mesh_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr height_pub_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomVisualizationNode>());
  rclcpp::shutdown();
  return 0;
}
