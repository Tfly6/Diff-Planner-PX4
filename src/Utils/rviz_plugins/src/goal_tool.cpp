/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <cmath>

#include "goal_tool.h"

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"

namespace rviz_plugins
{

Goal3DTool::Goal3DTool()
{
  shortcut_key_ = 'g';

  topic_property_ = new rviz_common::properties::StringProperty(
    "Topic",
    "goal",
    "The topic on which to publish navigation goals.",
    getPropertyContainer(),
    SLOT(updateTopic()),
    this);
}

void Goal3DTool::onInitialize()
{
  Pose3DTool::onInitialize();
  setName("3D Nav Goal");
  updateTopic();
}

void Goal3DTool::updateTopic()
{
  auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
  if (!ros_node_abstraction) {
    return;
  }

  raw_node_ = ros_node_abstraction->get_raw_node();
  pub_goal_ = raw_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
    topic_property_->getStdString(), rclcpp::QoS(1));
  pub_drone_id_goal_ = raw_node_->create_publisher<quadrotor_msgs::msg::GoalSet>(
    "/goal_with_id", rclcpp::QoS(1));
}

void Goal3DTool::onPoseSet(double x, double y, double z, double theta)
{
  if (!raw_node_) {
    updateTopic();
  }
  if (!raw_node_ || !pub_goal_ || !pub_drone_id_goal_) {
    return;
  }

  const std::string fixed_frame = context_->getFixedFrame().toStdString();
  geometry_msgs::msg::PoseStamped goal;
  goal.header.frame_id = fixed_frame;
  goal.header.stamp = raw_node_->now();
  goal.pose.position.x = x;
  goal.pose.position.y = y;
  goal.pose.position.z = z;
  goal.pose.orientation.x = 0.0;
  goal.pose.orientation.y = 0.0;
  goal.pose.orientation.z = std::sin(theta * 0.5);
  goal.pose.orientation.w = std::cos(theta * 0.5);
  pub_goal_->publish(goal);

  quadrotor_msgs::msg::GoalSet goal_with_id;
  goal_with_id.drone_id = 0;
  goal_with_id.goal[0] = static_cast<float>(x);
  goal_with_id.goal[1] = static_cast<float>(y);
  goal_with_id.goal[2] = static_cast<float>(z);
  pub_drone_id_goal_->publish(goal_with_id);
}

}  // namespace rviz_plugins

PLUGINLIB_EXPORT_CLASS(rviz_plugins::Goal3DTool, rviz_common::Tool)
