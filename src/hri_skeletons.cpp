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

#include "hri_rviz/hri_skeletons.hpp"

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "urdf/model.h"

#include "tf2_ros/transform_listener.h"

#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/file_picker_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/string_property.hpp"

namespace rviz_default_plugins
{
namespace displays
{

using rviz_common::properties::EnumProperty;
using rviz_common::properties::FilePickerProperty;
using rviz_common::properties::FloatProperty;
using rviz_common::properties::StatusProperty;
using rviz_common::properties::StringProperty;
using rviz_common::properties::Property;

void linkUpdaterStatusFunction(
  StatusProperty::Level level,
  const std::string & link_name,
  const std::string & text,
  SkeletonsDisplay * display)
{
  display->setStatus(level, QString::fromStdString(link_name), QString::fromStdString(text));
}

SkeletonsDisplay::SkeletonsDisplay()
: has_new_transforms_(false),
  time_since_last_transform_(0.0f),
  transformer_guard_(
    std::make_unique<rviz_default_plugins::transformation::TransformerGuard<
      rviz_default_plugins::transformation::TFFrameTransformer>>(this, "TF"))
{

  tf_prefix_property_ = new StringProperty(
    "TF Prefix", "",
    "Robot Model normally assumes the link name is the same as the tf frame name. "
    " This option allows you to set a prefix.  Mainly useful for multi-robot situations.",
    this, SLOT(updateTfPrefix()));

  hri_executor_ = rclcpp::executors::MultiThreadedExecutor::make_shared();
  hri_node_ = rclcpp::Node::make_shared("hri_node_hri_skeletons");
  async_client_node_ = rclcpp::Node::make_shared("async_client_node");
  hri_executor_->add_node(hri_node_);
  hri_executor_->add_node(async_client_node_);
  hri_listener_ = hri::HRIListener::create(hri_node_);

  visual_enabled_property_ = new Property(
    "Visual Enabled", true,
    "Whether to display the visual representation of the robot.",
    this, SLOT(updateVisualVisible()));

  collision_enabled_property_ = new Property(
    "Collision Enabled", false,
    "Whether to display the collision representation of the robot.",
    this, SLOT(updateCollisionVisible()));

  mass_properties_ = new Property("Mass Properties", QVariant(), "", this);
  mass_enabled_property_ = new Property(
    "Mass", false,
    "Whether to display the visual representation of the mass of each link.",
    mass_properties_, SLOT(updateMassVisible()), this);
  inertia_enabled_property_ = new Property(
    "Inertia", false,
    "Whether to display the visual representation of the inertia of each link.",
    mass_properties_, SLOT(updateInertiaVisible()), this);
  mass_properties_->collapse();

  update_rate_property_ = new FloatProperty(
    "Update Interval", 0,
    "Interval at which to update the links, in seconds. "
    " 0 means to update every update cycle.",
    this);
  update_rate_property_->setMin(0);

  alpha_property_ = new FloatProperty(
    "Alpha", 1,
    "Amount of transparency to apply to the links.",
    this, SLOT(updateAlpha()));
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);
}

SkeletonsDisplay::~SkeletonsDisplay() = default;

void SkeletonsDisplay::onInitialize()
{
  Display::onInitialize();

  updateVisualVisible();
  updateCollisionVisible();
  updateAlpha();

  transformer_guard_->initialize(context_);
}

void SkeletonsDisplay::updateAlpha()
{
  for (auto & human: humans_) {
    human.second->setAlpha(alpha_property_->getFloat());
  }
  context_->queueRender();
}

void SkeletonsDisplay::updateVisualVisible()
{
  for (auto & human: humans_) {
    human.second->setVisualVisible(visual_enabled_property_->getValue().toBool());
  }
  context_->queueRender();
}

void SkeletonsDisplay::updateCollisionVisible()
{
  for (auto & human: humans_) {
    human.second->setCollisionVisible(collision_enabled_property_->getValue().toBool());
  }
  context_->queueRender();
}

void SkeletonsDisplay::updateTfPrefix()
{
  clearStatuses();
  context_->queueRender();
}

void SkeletonsDisplay::updateMassVisible()
{
  for (auto & human: humans_) {
    human.second->setMassVisible(mass_enabled_property_->getValue().toBool());
  }
  context_->queueRender();
}

void SkeletonsDisplay::updateInertiaVisible()
{
  for (auto & human: humans_) {
    human.second->setInertiaVisible(inertia_enabled_property_->getValue().toBool());
  }
  context_->queueRender();
}

void SkeletonsDisplay::load_urdf(HumanPtr & human)
{
  if (!transformer_guard_->checkTransformer()) {
    return;
  }
  if (!human->initialized()) {
    return;
  }
  if (human->initialized()) {
    display_urdf_content(human);
  }
}

void SkeletonsDisplay::display_urdf_content(HumanPtr & human)
{
  urdf::Model descr;
  if (!descr.initString(human->description())) {
    clear();
    setStatus(
      StatusProperty::Error, QString::fromStdString(
        "URDF " + human->id()), "URDF failed Model parse");
    return;
  }

  setStatus(StatusProperty::Ok, QString::fromStdString("URDF " + human->id()), "URDF parsed OK");
  human->load(descr);
  std::stringstream ss;
  for (const auto & name_link_pair : human->getLinks()) {
    const std::string err = name_link_pair.second->getGeometryErrors();
    if (!err.empty()) {
      ss << "\n• for link '" << name_link_pair.first << "':\n" << err;
    }
  }
  if (ss.tellp()) {
    setStatus(
      StatusProperty::Error, "URDF",
      QString("Errors loading geometries:").append(ss.str().c_str()));
  }
  updateRobot(human);
}

void SkeletonsDisplay::updateRobot(HumanPtr & human)
{
  human->update(
    robot::TFLinkUpdater(
      context_->getFrameManager(),
      [this](auto arg1, auto arg2, auto arg3) {linkUpdaterStatusFunction(arg1, arg2, arg3, this);},
      tf_prefix_property_->getStdString()));
}

void SkeletonsDisplay::onEnable()
{
  for (auto & human: humans_) {
    load_urdf(human.second);
    human.second->setVisible(true);
  }
}

void SkeletonsDisplay::onDisable()
{
  Display::onDisable();
  for (auto & human: humans_) {
    human.second->setVisible(false);
  }
  clear();
}

void SkeletonsDisplay::updateBodies()
{
  auto bodies = hri_listener_->getBodies();
  // Add the newly detected bodies
  for (auto & body: bodies) {
    if (body.second->valid()) {
      auto body_id = body.first;
      auto body_ptr = body.second;
      auto human_it = humans_.find(body_id);
      auto body_description = body_ptr->bodyDescription();
      if ((human_it == humans_.end()) && body_description && !((*body_description).empty())) {
        auto insert_res = humans_.insert(
          std::pair<std::string, HumanPtr>(
            body_id, std::make_unique<Human>(
              scene_node_, context_, "body_" + body_id, this, body_id)));
        if (insert_res.second) {
          insert_res.first->second->setDescription(*body_ptr->bodyDescription());
          load_urdf(insert_res.first->second);
        }
      }
    }
  }
  // Remove the currently-not-detected bodies
  std::vector<std::string> bodies_to_remove;
  for (auto & human: humans_) {
    auto body_it = bodies.find(human.first);
    if (body_it == bodies.end()) {
      bodies_to_remove.push_back(human.first);
      human.second->hideLinks();
      human.second->disableLinkStatus(this);
      deleteStatusStd("URDF " + human.second->id());
    } else if (!human.second->initialized()) {
      load_urdf(human.second);
    }
  }
  for (auto & body_to_remove: bodies_to_remove) {
    humans_.erase(body_to_remove);
  }
}

void SkeletonsDisplay::update(float wall_dt, float ros_dt)
{
  if (!transformer_guard_->checkTransformer()) {
    return;
  }

  hri_executor_->spin_some();

  updateBodies();

  (void) ros_dt;
  time_since_last_transform_ += wall_dt;
  float rate = update_rate_property_->getFloat();
  bool update = rate < 0.0001f || time_since_last_transform_ >= rate * 1000000000;

  if (has_new_transforms_ || update) {
    for (auto & human: humans_) {
      updateRobot(human.second);
    }
    context_->queueRender();

    has_new_transforms_ = false;
    time_since_last_transform_ = 0.0f;
  }
}

void SkeletonsDisplay::fixedFrameChanged()
{
  has_new_transforms_ = true;
}

void SkeletonsDisplay::clear()
{
  for (auto & human: humans_) {
    human.second->clear();
  }
  clearStatuses();
}

void SkeletonsDisplay::reset()
{
  Display::reset();
  has_new_transforms_ = true;
}

}  // namespace displays
}  // namespace rviz_default_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_default_plugins::displays::SkeletonsDisplay, rviz_common::Display)
