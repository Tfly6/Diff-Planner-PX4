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

#include <OgrePlane.h>
#include <OgreQuaternion.h>
#include <OgreSceneNode.h>

#include "rviz_common/render_panel.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_rendering/objects/arrow.hpp"
#include "rviz_rendering/viewport_projection_finder.hpp"

#include "pose_tool.h"

namespace rviz_plugins
{

Pose3DTool::Pose3DTool()
  : rviz_common::Tool(), prev_angle_(0.0), init_z_(0.0), prev_z_(0.0)
{
}

Pose3DTool::~Pose3DTool()
{
}

void Pose3DTool::onInitialize()
{
  arrow_ = std::make_shared<rviz_rendering::Arrow>(scene_manager_, nullptr, 2.0f, 0.2f, 0.5f, 0.35f);
  arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
  arrow_->getSceneNode()->setVisible(false);
  projection_finder_ = std::make_shared<rviz_rendering::ViewportProjectionFinder>();
}

void Pose3DTool::activate()
{
  setStatus("Click and drag mouse to set position/orientation.");
  state_ = Position;
}

void Pose3DTool::deactivate()
{
  arrow_->getSceneNode()->setVisible(false);
  arrow_array_.clear();
}

int Pose3DTool::processMouseEvent(rviz_common::ViewportMouseEvent & event)
{
  int flags = 0;
  constexpr double z_scale = 50.0;
  constexpr double z_interval = 0.5;
  const Ogre::Quaternion orient_x(
    Ogre::Radian(Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Z);

  if (event.leftDown())
  {
    if (state_ != Position) {
      return flags;
    }

    auto xy_plane_intersection = projection_finder_->getViewportPointProjectionOnXYPlane(
      event.panel->getRenderWindow(), event.x, event.y);
    if (xy_plane_intersection.first)
    {
      pos_ = xy_plane_intersection.second;
      arrow_->setPosition(pos_);
      state_ = Orientation;
      flags |= Render;
    }
  }
  else if (event.type == QEvent::MouseMove && event.left())
  {
    if (state_ == Orientation)
    {
      auto xy_plane_intersection = projection_finder_->getViewportPointProjectionOnXYPlane(
        event.panel->getRenderWindow(), event.x, event.y);
      if (xy_plane_intersection.first)
      {
        const Ogre::Vector3 & cur_pos = xy_plane_intersection.second;
        const double angle = std::atan2(cur_pos.y - pos_.y, cur_pos.x - pos_.x);
        arrow_->getSceneNode()->setVisible(true);
        arrow_->setOrientation(Ogre::Quaternion(orient_x));
        if (event.right()) {
          state_ = Height;
        }
        init_z_ = pos_.z;
        prev_z_ = event.y;
        prev_angle_ = angle;
        flags |= Render;
      }
    }
    if (state_ == Height)
    {
      const double z = event.y;
      const double dz = z - prev_z_;
      prev_z_ = z;
      pos_.z -= dz / z_scale;
      arrow_->setPosition(pos_);
      arrow_array_.clear();
      const int cnt = static_cast<int>(std::ceil(std::fabs(init_z_ - pos_.z) / z_interval));
      for (int k = 0; k < cnt; ++k)
      {
        auto arrow = std::make_shared<rviz_rendering::Arrow>(
          scene_manager_, nullptr, 0.5f, 0.1f, 0.0f, 0.1f);
        arrow->setColor(0.0f, 1.0f, 0.0f, 1.0f);
        arrow->getSceneNode()->setVisible(true);
        Ogre::Vector3 arr_pos = pos_;
        arr_pos.z = init_z_ - ((init_z_ - pos_.z > 0) ? 1 : -1) * k * z_interval;
        arrow->setPosition(arr_pos);
        arrow->setOrientation(
          Ogre::Quaternion(Ogre::Radian(prev_angle_), Ogre::Vector3::UNIT_Z) *
          orient_x);
        arrow_array_.push_back(arrow);
      }
      flags |= Render;
    }
  }
  else if (event.leftUp())
  {
    if (state_ == Orientation || state_ == Height)
    {
      arrow_array_.clear();
      onPoseSet(pos_.x, pos_.y, pos_.z, prev_angle_);
      flags |= (Finished | Render);
    }
  }

  return flags;
}
}  // namespace rviz_plugins
