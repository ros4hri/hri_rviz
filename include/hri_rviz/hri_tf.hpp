/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2018, Bosch Software Innovations GmbH.
 * Copyright (c) 2024, PAL Robotics, S.L.
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

#ifndef RVIZ_TF_HRI_DISPLAY_H
#define RVIZ_TF_HRI_DISPLAY_H

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <OgreQuaternion.h>
#include <OgreVector3.h>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/exceptions.h"
#include "tf2/buffer_core.h"
#include "tf2/time.h"

#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/display.hpp"

#include "rviz_default_plugins/transformation/transformer_guard.hpp"
#include "rviz_default_plugins/transformation/tf_frame_transformer.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>

#include <hri_msgs/msg/ids_list.hpp>
#include <hri/hri.hpp>

namespace Ogre
{
class SceneNode;
}

namespace rviz_rendering
{
class Arrow;
class Axes;
class MovableText;
}

namespace rviz_common
{

namespace properties
{
class BoolProperty;
class FloatProperty;
class QuaternionProperty;
class StringProperty;
class VectorProperty;
}  // namespace properties
}  // namespace rviz_common

namespace rviz_default_plugins
{
namespace displays
{

namespace hri_tools
{
class FrameInfo;
class FrameSelectionHandler;
typedef std::set<hri_tools::FrameInfo *> S_FrameInfo;
typedef std::shared_ptr<FrameSelectionHandler> FrameSelectionHandlerPtr;
}

static std::vector<std::string> skeleton_components = {"body", "head", "torso", "waist", "p_head",
  "y_head", "l_ankle", "l_elbow", "l_hip", "l_knee",
  "l_p_hip", "l_p_shoulder", "l_shoulder", "l_wrist", "l_y_hip",
  "l_y_shoulder", "r_ankle", "r_elbow", "r_hip", "r_knee",
  "r_p_hip", "r_p_shoulder", "r_shoulder", "r_wrist", "r_y_hip", "r_y_shoulder"};

/** @brief Displays a visual representation of the TF hierarchy. */
class RVIZ_DEFAULT_PLUGINS_PUBLIC TFHRIDisplay : public rviz_common::Display
{
  Q_OBJECT

public:
  TFHRIDisplay();

  ~TFHRIDisplay() override;

  void update(float wall_dt, float ros_dt) override;

protected:
  void onInitialize() override;
  void load(const rviz_common::Config & config) override;
  void fixedFrameChanged() override;
  void reset() override;

private Q_SLOTS:
  void updateShowAxes();
  void updateShowArrows();
  void updateShowNames();
  void allEnabledChanged();
  void showFacesChanged();
  void showGazesChanged();
  void showBodiesChanged();

private:
  void updateFrames();
  hri_tools::FrameInfo * createFrame(const std::string & frame);
  void updateFrame(hri_tools::FrameInfo * frame);
  void deleteFrame(hri_tools::FrameInfo * frame, bool delete_properties);
  hri_tools::FrameInfo * getFrameInfo(const std::string & frame);
  void clear();

  void onEnable() override;
  void onDisable() override;

  static bool startsWith(const std::string & a, const std::string & b)
  {
    // Checks whether a starts with b or not
    if (a.size() < b.size()) {
      return false;
    }
    auto partial_a = a.substr(0, b.size());
    return partial_a == b;
  }

  static bool hasId(const std::string & a, const std::string & b)
  {
    if (a.size() < b.size()) {
      return false;
    }
    if (a.size() < 5) {
      return false;
    }
    std::string id = a.substr(a.size() - 5, 5);
    return id == b;
  }

  Ogre::SceneNode * root_node_;
  Ogre::SceneNode * names_node_;
  Ogre::SceneNode * arrows_node_;
  Ogre::SceneNode * axes_node_;

  typedef std::map<std::string, hri_tools::FrameInfo *> M_FrameInfo;
  M_FrameInfo frames_;

  typedef std::map<std::string, bool> M_EnabledState;
  M_EnabledState frame_config_enabled_state_;

  float update_timer_;

  rviz_common::properties::BoolProperty * show_names_property_;
  rviz_common::properties::BoolProperty * show_arrows_property_;
  rviz_common::properties::BoolProperty * show_axes_property_;
  rviz_common::properties::FloatProperty * update_rate_property_;
  rviz_common::properties::FloatProperty * frame_timeout_property_;
  rviz_common::properties::BoolProperty * all_enabled_property_;
  rviz_common::properties::BoolProperty * show_faces_property_;
  rviz_common::properties::BoolProperty * show_gazes_property_;
  rviz_common::properties::BoolProperty * show_bodies_property_;

  rviz_common::properties::FloatProperty * scale_property_;

  rviz_common::properties::Property * frames_category_;
  rviz_common::properties::Property * tree_category_;

  bool changing_single_frame_enabled_state_;
  bool show_faces_;
  bool show_gazes_;
  bool show_bodies_;

  std::unique_ptr<rviz_default_plugins::transformation::TransformerGuard<
      rviz_default_plugins::transformation::TFFrameTransformer>> transformer_guard_;

  void updateRelativePositionAndOrientation(
    const hri_tools::FrameInfo * frame, std::shared_ptr<tf2::BufferCore> tf_buffer) const;

  void logTransformationException(
    const std::string & parent_frame,
    const std::string & child_frame,
    const std::string & message = "") const;

  void updateParentArrowIfTransformExists(
    hri_tools::FrameInfo * frame,
    const Ogre::Vector3 & position) const;

  bool hasNoTreePropertyOrParentChanged(
    const hri_tools::FrameInfo * frame, const std::string & old_parent) const;
  void updateParentTreeProperty(hri_tools::FrameInfo * frame) const;

  void deleteObsoleteFrames(std::set<hri_tools::FrameInfo *> & current_frames);
  hri_tools::S_FrameInfo createOrUpdateFrames(const std::vector<std::string> & frames);

  rclcpp::Executor::SharedPtr hri_executor_;
  rclcpp::Node::SharedPtr hri_node_;
  rclcpp::Node::SharedPtr async_client_node_;
  std::shared_ptr<hri::HRIListener> hri_listener_;

  std::shared_ptr<rclcpp::AsyncParametersClient> apc_;

  friend class hri_tools::FrameInfo;
};

}  // namespace displays
}  // namespace rviz_default_plugins

#endif  // RVIZ_TF_DISPLAY_H
