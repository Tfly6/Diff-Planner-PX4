/*
 * Copyright 2019-2020 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************
 *  v0.1.0: drwnz (david.wong@tier4.jp) *
 *
 * selected_points_publisher.hpp
 *
 *  Created on: December 5th 2019
 */

#ifndef SELECTED_POINTS_PUBLISHER__SELECTED_POINTS_PUBLISHER_HPP_
#define SELECTED_POINTS_PUBLISHER__SELECTED_POINTS_PUBLISHER_HPP_

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>
#endif

#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/node.hpp"

#include "rviz_default_plugins/tools/select/selection_tool.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"

namespace rviz_plugin_selected_points_publisher
{

class SelectedPointsPublisher : public rviz_default_plugins::tools::SelectionTool
{
  Q_OBJECT
public:
  SelectedPointsPublisher();
  ~SelectedPointsPublisher() override;
  void onInitialize() override;
  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;
  int processKeyEvent(QKeyEvent * event, rviz_common::RenderPanel * panel) override;

public Q_SLOTS:
  void updateTopic();

protected:
  int processSelectedArea();
  rclcpp::Node::SharedPtr raw_node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_selected_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;

  std::string tf_frame_;
  std::string selected_drones_topic_, goal_topic_;

  std::vector<geometry_msgs::msg::PoseStamped> selected_drones_;
  std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;

  bool selecting_;
  int num_selected_points_;
};
}  // namespace rviz_plugin_selected_points_publisher

#endif  // SELECTED_POINTS_PUBLISHER__SELECTED_POINTS_PUBLISHER_HPP_
