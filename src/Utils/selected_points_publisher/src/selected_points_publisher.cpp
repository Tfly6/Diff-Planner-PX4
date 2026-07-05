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
 * selected_points_publisher.cpp
 *
 *  Created on: December 5th 2019
 */

#include "selected_points_publisher/selected_points_publisher.hpp"

#include <QKeyEvent>
#include <QVariant>

#include "pluginlib/class_list_macros.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/property_tree_model.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/render_panel.hpp"

namespace rviz_plugin_selected_points_publisher
{
  SelectedPointsPublisher::SelectedPointsPublisher()
  : selecting_(false), num_selected_points_(0)
  {
  }

  SelectedPointsPublisher::~SelectedPointsPublisher()
  {
  }

  void SelectedPointsPublisher::onInitialize()
  {
    rviz_default_plugins::tools::SelectionTool::onInitialize();
    projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
    updateTopic();
  }

  void SelectedPointsPublisher::updateTopic()
  {
    auto ros_node_abstraction = context_->getRosNodeAbstraction().lock();
    if (!ros_node_abstraction) {
      return;
    }

    raw_node_ = ros_node_abstraction->get_raw_node();
    tf_frame_ = "base_link";
    selected_drones_topic_ = std::string("/rviz_selected_drones");
    goal_topic_ = std::string("/goal");

    rviz_selected_publisher_ =
      raw_node_->create_publisher<geometry_msgs::msg::PoseStamped>(selected_drones_topic_, rclcpp::QoS(100));
    goal_publisher_ =
      raw_node_->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, rclcpp::QoS(100));
    num_selected_points_ = 0;
  }

  int SelectedPointsPublisher::processKeyEvent(QKeyEvent * event, rviz_common::RenderPanel * panel)
  {
    int flags = rviz_default_plugins::tools::SelectionTool::processKeyEvent(event, panel);
    if (event->type() == QEvent::KeyPress)
    {
      if (event->key() == 'c' || event->key() == 'C')
      {
        selecting_ = false;
        selected_drones_.clear();
        num_selected_points_ = 0;
      }
      else if (event->key() == 'p' || event->key() == 'P')
      {
        for (size_t i = 0; i < selected_drones_.size(); ++i)
        {
          rviz_selected_publisher_->publish(selected_drones_[i]);
        }
      }
    }

    return flags;
  }

  int SelectedPointsPublisher::processMouseEvent(rviz_common::ViewportMouseEvent & event)
  {
    int flags = rviz_default_plugins::tools::SelectionTool::processMouseEvent(event);
    if (event.alt())
    {
      selecting_ = false;
    }
    else
    {
      if (event.leftDown())
      {
        selecting_ = true;
      }
      if (event.rightUp()) // Publish immediately!
      {
        auto xy_plane_intersection = projection_finder_->getViewportPointProjectionOnXYPlane(
          event.panel->getRenderWindow(), event.x, event.y);
        if (xy_plane_intersection.first)
        {
          this->processSelectedArea();
          for (size_t i = 0; i < selected_drones_.size(); ++i)
          {
            rviz_selected_publisher_->publish(selected_drones_[i]);
          }

          geometry_msgs::msg::PoseStamped goal_msg;
          goal_msg.header.frame_id = std::string("world");
          goal_msg.header.stamp = raw_node_->now();
          goal_msg.pose.position.x = xy_plane_intersection.second.x;
          goal_msg.pose.position.y = xy_plane_intersection.second.y;
          goal_msg.pose.position.z = 1.0;
          goal_publisher_->publish(goal_msg);
        }
      }
    }

    if (selecting_)
    {
      if (event.leftUp())
      {
        this->processSelectedArea();
      }
    }
    return flags;
  }

  int SelectedPointsPublisher::processSelectedArea()
  {
    auto selection_manager = context_->getSelectionManager();
    auto * model = selection_manager->getPropertyModel();

    selected_drones_.clear();
    int i = 0;
    while (model->hasIndex(i, 0))
    {

      QModelIndex child_index = model->index(i, 0);

      auto * child = model->getProp(child_index);
      auto * subchild = dynamic_cast<rviz_common::properties::VectorProperty *>(child->childAt(0));
      if (subchild == nullptr) {
        ++i;
        continue;
      }
      Ogre::Vector3 point_data = subchild->getVector();
      std::string name = child->getNameStd();

      if (name.substr(0, 12) == std::string("Marker drone"))
      {
        geometry_msgs::msg::PoseStamped droneX_msg;
        droneX_msg.header.frame_id = std::string("drone_") + name.substr(13, 20);
        droneX_msg.header.stamp = raw_node_->now();
        droneX_msg.pose.position.x = point_data.x;
        droneX_msg.pose.position.y = point_data.y;
        droneX_msg.pose.position.z = point_data.z;

        selected_drones_.push_back(droneX_msg);
      }

      i++;
    }
    num_selected_points_ = i;

    return 0;
  }
} // namespace rviz_plugin_selected_points_publisher

PLUGINLIB_EXPORT_CLASS(
  rviz_plugin_selected_points_publisher::SelectedPointsPublisher,
  rviz_common::Tool)
