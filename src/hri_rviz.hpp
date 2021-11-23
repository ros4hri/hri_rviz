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

#ifndef RVIZ_HRI_BODIES_DISPLAY_H
#define RVIZ_HRI_BODIES_DISPLAY_H

#include <OGRE/OgreVector3.h>
#include <hri_msgs/IdsList.h>
#include <rviz/display.h>
#include <rviz/message_filter_display.h>

#include <map>

namespace rviz {
class FloatProperty;
class Robot;

/**
 * \class HriBodiesDisplay
 * \brief Uses a robot xml description to display the pieces of a robot at the
 * transforms broadcast by rosTF
 */
class HriBodiesDisplay : public MessageFilterDisplay<hri_msgs::IdsList> {
    Q_OBJECT
   public:
    HriBodiesDisplay();
    ~HriBodiesDisplay() override;

    // Overrides from Display
    void onInitialize() override;
    void update(float wall_dt, float ros_dt) override;
    void fixedFrameChanged() override;
    void reset() override;
    using Display::load;

    void clear();

   private Q_SLOTS:
    void updateAlpha();

   protected:
    /** @brief Loads a URDF from the ros-param named by our
     * "Robot Description" property, iterates through the links, and
     * loads any necessary models. */
    virtual void load(const std::string& body_id);

    virtual void processMessage(const hri_msgs::IdsListConstPtr& msg) override;

    // overrides from Display
    void onEnable() override;
    void onDisable() override;

    std::map<std::string, Robot*>
        bodies_;  ///< Handles actually drawing the humans

    bool has_new_transforms_;  ///< Callback sets this to tell our update
                               ///< function it needs to update the
                               /// transforms

    float time_since_last_transform_;

    std::map<std::string, std::string> body_urdfs_;

    FloatProperty* alpha_property_;
};

}  // namespace rviz

#endif

