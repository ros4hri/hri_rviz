/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * Copyright (c) 2021, PAL Robotics
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

#include "hri_rviz.hpp"

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
                               HriBodiesDisplay* display) {
    display->setStatus(level, QString::fromStdString(link_name),
                       QString::fromStdString(text));
}

HriBodiesDisplay::HriBodiesDisplay()
    : Display(), has_new_transforms_(false), time_since_last_transform_(0.0f) {
    alpha_property_ = new FloatProperty(
        "Alpha", 1, "Amount of transparency to apply to the links.", this,
        SLOT(updateAlpha()));
    alpha_property_->setMin(0.0);
    alpha_property_->setMax(1.0);
}

HriBodiesDisplay::~HriBodiesDisplay() {
    if (initialized()) {
        delete robot_;
    }
}

void HriBodiesDisplay::onInitialize() {
    robot_ = new Robot(scene_node_, context_,
                       "Robot: " + getName().toStdString(), this);

    updateAlpha();
}

void HriBodiesDisplay::updateAlpha() {
    robot_->setAlpha(alpha_property_->getFloat());
    context_->queueRender();
}

// void HriBodiesDisplay::updateRobotDescription() {
//    if (isEnabled()) load();
//}

void HriBodiesDisplay::load() {
    clearStatuses();
    context_->queueRender();

    std::string content;
    try {
        if (!update_nh_.getParam("body_test_description", content)) {
            clear();
            setStatus(StatusProperty::Error, "URDF",
                      QString("Parameter [%1] does not exist, and was not "
                              "found by searchParam()")
                          .arg("body_test_description"));
            // try again in a second
            QTimer::singleShot(1000, this, SLOT(updateRobotDescription()));
            return;
        }
    } catch (const ros::InvalidNameException& e) {
        clear();
        setStatus(StatusProperty::Error, "URDF",
                  QString("Invalid parameter name: %1.\n%2")
                      .arg("body_test_description", e.what()));
        return;
    }

    if (content.empty()) {
        clear();
        setStatus(StatusProperty::Error, "URDF", "URDF is empty");
        return;
    }

    if (content == robot_description_) {
        return;
    }

    robot_description_ = content;

    urdf::Model descr;
    if (!descr.initString(robot_description_)) {
        clear();
        setStatus(StatusProperty::Error, "URDF", "Failed to parse URDF model");
        return;
    }

    setStatus(StatusProperty::Ok, "URDF", "URDF parsed OK");
    robot_->load(descr);
    std::stringstream ss;
    for (const auto& name_link_pair : robot_->getLinks()) {
        // const std::string& err = name_link_pair.second->getGeometryErrors();
        // if (!err.empty())
        //  ss << "\n• for link '" << name_link_pair.first << "':\n" << err;
    }
    if (ss.tellp())
        setStatus(
            StatusProperty::Error, "URDF",
            QString("Errors loading geometries:").append(ss.str().c_str()));

    robot_->update(TFLinkUpdater(
        context_->getFrameManager(),
        boost::bind(linkUpdaterStatusFunction, _1, _2, _3, this)));
}

void HriBodiesDisplay::onEnable() {
    load();
    robot_->setVisible(true);
}

void HriBodiesDisplay::onDisable() {
    robot_->setVisible(false);
    clear();
}

void HriBodiesDisplay::update(float wall_dt, float /*ros_dt*/) {
    time_since_last_transform_ += wall_dt;

    robot_->update(TFLinkUpdater(
        context_->getFrameManager(),
        boost::bind(linkUpdaterStatusFunction, _1, _2, _3, this)));
    context_->queueRender();

    has_new_transforms_ = false;
    time_since_last_transform_ = 0.0f;
}

void HriBodiesDisplay::fixedFrameChanged() { has_new_transforms_ = true; }

void HriBodiesDisplay::clear() {
    robot_->clear();
    clearStatuses();
    robot_description_.clear();
}

void HriBodiesDisplay::reset() {
    Display::reset();
    has_new_transforms_ = true;
}

}  // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::HriBodiesDisplay, rviz::Display)

