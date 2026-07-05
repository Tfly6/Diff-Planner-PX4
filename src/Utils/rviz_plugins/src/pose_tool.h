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

#ifndef RVIZ_PLUGINS__POSE_TOOL_H_
#define RVIZ_PLUGINS__POSE_TOOL_H_

#include <memory>
#include <vector>

#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include "rviz_common/tool.hpp"

namespace rviz_rendering
{
class Arrow;
class ViewportProjectionFinder;
}  // namespace rviz_rendering

namespace rviz_plugins
{

class Pose3DTool : public rviz_common::Tool
{
public:
  Pose3DTool();
  ~Pose3DTool() override;

  void onInitialize() override;

  void activate() override;
  void deactivate() override;

  int processMouseEvent(rviz_common::ViewportMouseEvent & event) override;

protected:
  virtual void onPoseSet(double x, double y, double z, double theta) = 0;

  std::shared_ptr<rviz_rendering::Arrow> arrow_;
  std::vector<std::shared_ptr<rviz_rendering::Arrow>> arrow_array_;
  std::shared_ptr<rviz_rendering::ViewportProjectionFinder> projection_finder_;

  enum State
  {
    Position,
    Orientation,
    Height
  };
  State state_;

  Ogre::Vector3 pos_;
  double prev_angle_;
  double init_z_;
  double prev_z_;
};

}  // namespace rviz_plugins

#endif  // RVIZ_PLUGINS__POSE_TOOL_H_
