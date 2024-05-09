/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#ifndef RVIZ_HRI_SKELETONS_H
#define RVIZ_HRI_SKELETONS_H

#include <map>
#include <memory>
#include <string>
#include <future>
#include <chrono>

#include <OgreVector.h>

#include "rviz_common/display.hpp"

#include "rviz_default_plugins/transformation/transformer_guard.hpp"
#include "rviz_default_plugins/transformation/tf_frame_transformer.hpp"
#include "rviz_default_plugins/visibility_control.hpp"

#include <hri_msgs/msg/ids_list.hpp>
#include <hri/hri.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/parameter_client.hpp>

#include "rviz_default_plugins/robot/robot.hpp"
#include "rviz_default_plugins/robot/robot_link.hpp"
#include "rviz_default_plugins/robot/tf_link_updater.hpp"

#include "rviz_common/properties/property.hpp"

namespace Ogre
{
class Entity;
class SceneNode;
}

namespace rviz_rendering
{
class Axes;
}

namespace rviz_common
{
namespace properties
{
class EnumProperty;
class FilePickerProperty;
class FloatProperty;
class Property;
class StringProperty;
}  // namespace properties
}  // namespace rviz_common

namespace rviz_default_plugins
{

namespace robot
{
class Robot;
}


namespace displays
{

class Human;

typedef std::unique_ptr<Human> HumanPtr;

static const std::string HUMAN_MODEL_PREFIX = "human_description_";

/**
 * \class SkeletonsDisplay
 */
class RVIZ_DEFAULT_PLUGINS_PUBLIC SkeletonsDisplay : public
  rviz_common::Display
{
  Q_OBJECT

public:
  SkeletonsDisplay();
  ~SkeletonsDisplay() override;

  // Overrides from Display
  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void fixedFrameChanged() override;
  void reset() override;

  void clear();

private Q_SLOTS:
  void updateVisualVisible();
  void updateCollisionVisible();
  void updateTfPrefix();
  void updateAlpha();
  void updateMassVisible();
  void updateInertiaVisible();

protected:
  virtual void load_urdf(HumanPtr & human);
  void display_urdf_content(HumanPtr & human);
  void updateRobot(HumanPtr & human);
  void updateBodies();

  // overrides from Display
  void onEnable() override;
  void onDisable() override;

  std::map<std::string, HumanPtr> humans_; // what we introduce to draw humans

  bool has_new_transforms_;      ///< Callback sets this to tell our update function
  ///< it needs to update the transforms

  float time_since_last_transform_;

  rviz_common::properties::Property * visual_enabled_property_;
  rviz_common::properties::Property * collision_enabled_property_;
  rviz_common::properties::FloatProperty * update_rate_property_;
  rviz_common::properties::FloatProperty * alpha_property_;
  rviz_common::properties::StringProperty * tf_prefix_property_;

  rviz_common::properties::Property * mass_properties_;
  rviz_common::properties::Property * mass_enabled_property_;
  rviz_common::properties::Property * inertia_enabled_property_;

  std::unique_ptr<rviz_default_plugins::transformation::TransformerGuard<
      rviz_default_plugins::transformation::TFFrameTransformer>> transformer_guard_;

  rclcpp::Executor::SharedPtr hri_executor_;
  rclcpp::Node::SharedPtr hri_node_;
  rclcpp::Node::SharedPtr async_client_node_;
  std::shared_ptr<hri::HRIListener> hri_listener_;
};

class Human : public robot::Robot
{
  std::string description_;
  bool initialized_;
  std::string id_; // debug

public:
  Human(
    Ogre::SceneNode * root_node,
    rviz_common::DisplayContext * context,
    const std::string & name,
    rviz_common::properties::Property * parent_property,
    std::string id)
  : robot::Robot(root_node, context, name, parent_property),
    initialized_(false),
    id_(id)
  {}

  void setDescription(const std::string & description)
  {
    description_ = description;
    if (description_.empty()) {
      initialized_ = false;
    } else {
      initialized_ = true;
    }
  }

  bool initialized() const
  {
    return initialized_;
  }

  std::string description() const
  {
    return description_;
  }

  std::string id() const
  {
    return id_;
  }

  void hideLinks()
  {
    this->link_tree_->hide();
  }

  // debug
  void disableLinkStatus(SkeletonsDisplay * display) const
  {
    for (auto & link_map_entry : this->links_) {
      display->deleteStatusStd(link_map_entry.first);
    }
  }
};

}  // namespace displays
}  // namespace rviz_default_plugins
#endif  // RVIZ_HRI_SKELETONS_H
