// Copyright 2021 PAL Robotics S.L.
// Copyright 2008, Willow Garage, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage, Inc., PAL Robotics S.L. nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "hri_skeletons.hpp"

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <rviz/display_context.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/property.h>
#include <rviz/properties/string_property.h>
#include <rviz/robot/robot.h>
#include <rviz/robot/robot_link.h>
#include <rviz/robot/tf_link_updater.h>
#include <urdf/model.h>

#include <QTimer>

namespace rviz {
void linkUpdaterStatusFunction(StatusProperty::Level level,
                               const std::string& link_name,
                               const std::string& text,
                               HumansModelDisplay* display) {
  display->setStatus(level, QString::fromStdString(link_name),
                     QString::fromStdString(text));
}

HumansModelDisplay::HumansModelDisplay()
    : Display(), has_new_transforms_(false), time_since_last_transform_(0.0f) {
  visual_enabled_property_ =
      new Property("Visual Enabled", true,
                   "Whether to display the visual representation of the robot.",
                   this, SLOT(updateVisualVisible()));

  collision_enabled_property_ = new Property(
      "Collision Enabled", false,
      "Whether to display the collision representation of the robot.", this,
      SLOT(updateCollisionVisible()));

  update_rate_property_ =
      new FloatProperty("Update Interval", 0,
                        "Interval at which to update the links, in seconds. "
                        "0 means to update every update cycle.",
                        this);

  update_rate_property_->setMin(0);

  alpha_property_ = new FloatProperty(
      "Alpha", 1, "Amount of transparency to apply to the links.", this,
      SLOT(updateAlpha()));
  alpha_property_->setMin(0.0);
  alpha_property_->setMax(1.0);

  tf_prefix_property_ =
      new StringProperty("TF Prefix", "",
                         "Robot Model normally assumes the link name is the "
                         "same as the tf frame name. "
                         " This option allows you to set a prefix.  Mainly "
                         "useful for multi-robot situations.",
                         this, SLOT(updateTfPrefix()));

  idsSub_ = update_nh_.subscribe("/humans/bodies/tracked", 1,
                                 &HumansModelDisplay::idsCallback, this);

  pluginEnabled_ = true;
}

HumansModelDisplay::~HumansModelDisplay() {
  if (initialized()) {
    for (std::map<std::string, rviz::RobotPtr>::iterator it = humans_.begin();
         it != humans_.end(); it++)
      it->second = nullptr;
  }
}

void HumansModelDisplay::onInitialize() {
  updateVisualVisible();
  updateCollisionVisible();
  updateAlpha();
}

void HumansModelDisplay::updateAlpha() {
  for (std::map<std::string, rviz::RobotPtr>::iterator it = humans_.begin();
       it != humans_.end(); it++)
    if(it->second != nullptr)
      it->second->setAlpha(alpha_property_->getFloat());
  context_->queueRender();
}

void HumansModelDisplay::updateVisualVisible() {
  for (std::map<std::string, rviz::RobotPtr>::iterator it = humans_.begin();
       it != humans_.end(); it++)
    if(it->second != nullptr)
      it->second->setVisualVisible(visual_enabled_property_->getValue().toBool());
  context_->queueRender();
}

void HumansModelDisplay::updateCollisionVisible() {
  for (std::map<std::string, rviz::RobotPtr>::iterator it = humans_.begin();
       it != humans_.end(); it++)
    if(it->second != nullptr)
      it->second->setVisualVisible(
          collision_enabled_property_->getValue().toBool());
  context_->queueRender();
}

void HumansModelDisplay::updateTfPrefix() {
  clearStatuses();
  context_->queueRender();
}

void HumansModelDisplay::initializeRobot(
    std::map<std::string, rviz::RobotPtr>::iterator it) {
  context_->queueRender();

  std::string description = "human_description_" + (it->first);
  std::string content;

  try {
    if (!update_nh_.getParam(description,
                             content))  // In content we get the string
                                        // representing the urdf model
    {
      std::string loc;
      if (update_nh_.searchParam(description, loc)){
        update_nh_.getParam(loc, content);
      }
      else {
        clear();
        setStatus(StatusProperty::Warn, "URDF",
                  QString("Parameter [%1] does not exist, and was not found by "
                          "searchParam()")
                      .arg(QString::fromStdString(description)));
        return;
      }
    }
  } catch (const ros::InvalidNameException& e) {
    clear();
    setStatus(StatusProperty::Warn, "URDF",
              QString("Invalid parameter name: %1.\n%2")
                  .arg(QString::fromStdString(description), e.what()));
    return;
  }

  if (content.empty()) {
    clear();
    setStatus(StatusProperty::Warn, "URDF", "URDF is empty");
    return;
  }

  std::string robot_description = content;

  urdf::Model descr;
  if (!descr.initString(robot_description)) {
    clear();
    setStatus(StatusProperty::Warn, "URDF", "Failed to parse URDF model");
    return;
  }

  it->second = RobotPtr((new Robot(scene_node_, context_, "Human: " + it->first, this)));

  setStatus(StatusProperty::Ok, "URDF", "URDFs parsed OK");
  it->second->load(descr);

  it->second->update(
      TFLinkUpdater(context_->getFrameManager(),
                    boost::bind(linkUpdaterStatusFunction, _1, _2, _3, this),
                    tf_prefix_property_->getStdString()));
}

void HumansModelDisplay::onEnable() {
  for (std::map<std::string, rviz::RobotPtr>::iterator it = humans_.begin();
       it != humans_.end(); it++)
    if(it->second != nullptr)
      it->second->setVisible(true);
  pluginEnabled_ = true;
}

void HumansModelDisplay::onDisable() {
  // robot_->setVisible(false);
  for (std::map<std::string, rviz::RobotPtr>::iterator it = humans_.begin();
       it != humans_.end(); it++)
    if(it->second != nullptr)
      it->second->setVisible(false);
  pluginEnabled_ = false;
  //clear();
}

void HumansModelDisplay::update(float wall_dt, float /*ros_dt*/) {
  time_since_last_transform_ += wall_dt;
  float rate = update_rate_property_->getFloat();
  bool update = rate < 0.0001f || time_since_last_transform_ >= rate;

  if (has_new_transforms_ || update) {
    for (std::map<std::string, rviz::RobotPtr>::iterator it = humans_.begin(); it != humans_.end(); it++){
      if(it->second != nullptr){
        it->second->update(TFLinkUpdater(
            context_->getFrameManager(),
            boost::bind(linkUpdaterStatusFunction, _1, _2, _3, this),
            tf_prefix_property_->getStdString()));
      }
    }
    context_->queueRender();

    has_new_transforms_ = false;
    time_since_last_transform_ = 0.0f;
  }
}

void HumansModelDisplay::fixedFrameChanged() { has_new_transforms_ = true; }

void HumansModelDisplay::clear() {
  // robot_->clear();
  for (std::map<std::string, rviz::RobotPtr>::iterator it = humans_.begin();
       it != humans_.end(); it++)
    if(it->second != nullptr)
      it->second->clear();
  clearStatuses();
  robot_description_.clear();
}

void HumansModelDisplay::reset() {
  Display::reset();
  has_new_transforms_ = true;
}

void HumansModelDisplay::idsCallback(const hri_msgs::IdsListConstPtr& msg) {
  if (ros::ok() && pluginEnabled_) {
    ids_ = msg->ids;

    // Check for bodies that are no more in the list
    // Remove them from the map0

    std::map<std::string, rviz::RobotPtr>::iterator itH;
    for (itH = humans_.begin(); itH != humans_.end();) {
      if (std::find(ids_.begin(), ids_.end(), itH->first) == ids_.end()) {
        if(itH->second)
          itH->second = nullptr;
        humans_.erase((itH++)->first);
      } else
        ++itH;
    }

    // Check for new faces
    // Create a bounding box message and insert it in the map

  
    for (const auto& id : ids_) {
      auto human = humans_.find(id);
      if (human == humans_.end()) {
        std::string human_description = "human_description_"+id;
        if(update_nh_.hasParam(human_description)){
          auto ins = humans_.insert(std::pair<std::string, rviz::RobotPtr>(id, nullptr));
          ROS_WARN("Initializing robot");
          if (ins.second) initializeRobot(ins.first); // Maybe I could remove this
        }    
      }
    }
  }
}

}  // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::HumansModelDisplay, rviz::Display)
