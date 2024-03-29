// Copyright 2021 PAL Robotics S.L.
// Copyright 2012, Willow Garage, Inc.
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

#ifndef RVIZ_HRI_HUMANS_H
#define RVIZ_HRI_HUMANS_H

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreRenderTargetListener.h>
#include <OGRE/OgreSharedPtr.h>
#include <cv_bridge/cv_bridge.h>
#include <hri_msgs/IdsList.h>
#include <hri_msgs/NormalizedPointOfInterest2D.h>

#include <QObject>
#include <map>
#include <string>
#include <vector>

#include "hri/hri.h"
#include "ros/ros.h"
#include "rviz/image/image_display_base.h"
#include "rviz/image/ros_image_texture.h"
#include "rviz/properties/bool_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"
#include "rviz/render_panel.h"
#endif

namespace Ogre {
class SceneNode;
class Rectangle2D;
}  // namespace Ogre

namespace rviz {
class HumansDisplay : public ImageDisplayBase {
  Q_OBJECT
 public:
  HumansDisplay();
  ~HumansDisplay() override;

  // Overrides from Display
  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

 public Q_SLOTS:
  virtual void updateNormalizeOptions();
  void updateShowFaces();
  void updateShowBodies();
  void updateShowFacialLandmarks();
  void updateShowSkeletons();   

 protected:
  // overrides from Display
  void onEnable() override;
  void onDisable() override;

  // skeleton drawing function
  void drawSkeleton(std::string id, int width, int height, std::vector<hri_msgs::NormalizedPointOfInterest2D>& skeleton);

  /* This is called by incomingMessage(). */
  void processMessage(const sensor_msgs::Image::ConstPtr& msg) override;

  Ogre::SceneManager* img_scene_manager_;

  ROSImageTexture texture_;

  RenderPanel* render_panel_;

  // ros::NodeHandle nh_;

 private:
  Ogre::SceneNode* img_scene_node_;
  Ogre::Rectangle2D* screen_rect_;
  Ogre::MaterialPtr material_;

  BoolProperty* normalize_property_;
  BoolProperty* show_faces_property_;
  BoolProperty* show_facial_landmarks_property_;
  BoolProperty* show_bodies_property_;
  BoolProperty* show_skeletons_property_;
  FloatProperty* min_property_;
  FloatProperty* max_property_;
  IntProperty* median_buffer_size_property_;
  bool got_float_image_;
  bool show_faces_, show_facial_landmarks_, show_bodies_, show_skeletons_;

  hri::HRIListener hri_listener;
  cv_bridge::CvImagePtr cvBridge_;
};

}  // namespace rviz

#endif
