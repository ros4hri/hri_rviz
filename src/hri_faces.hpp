/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * Copyright (c) 2021, PAL Robotics
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

#ifndef RVIZ_IMAGE_DISPLAY_H
#define RVIZ_IMAGE_DISPLAY_H

#ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QObject>

#include <OGRE/OgreMaterial.h>
#include <OGRE/OgreRenderTargetListener.h>
#include <OGRE/OgreSharedPtr.h>

#include "rviz/image/image_display_base.h"
#include "rviz/image/ros_image_texture.h"
#include "rviz/render_panel.h"

#include "rviz/properties/bool_property.h"
#include "rviz/properties/float_property.h"
#include "rviz/properties/int_property.h"

#include <vector>
#include <string>
#include <hri_msgs/IdsList.h>
#include <hri_msgs/RegionOfInterestStamped.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <map>
#include "ros/ros.h"
#include <cv_bridge/cv_bridge.h>
#endif


namespace Ogre
{
class SceneNode;
class Rectangle2D;
} // namespace Ogre

namespace rviz
{

class FacesDisplay;

class BoundingBox
{
  uint32_t x, y, width, height;
  ros::Subscriber roi_sub_;
  int R_, G_, B_;

public:

  BoundingBox(const std::string& id, ros::NodeHandle& nh, int R, int G, int B, FacesDisplay* obj);

  ~BoundingBox(){};

  inline int getR() const {return R_;}
  inline int getG() const {return G_;}
  inline int getB() const {return B_;}

  inline cv::Scalar getRGB() const {return cv::Scalar(R_, G_, B_);}

  inline bool bbInitialized() const {return width>0;}

  inline cv::Rect getRect() const {return cv::Rect(int(x), int(y), int(width), int(height));}
  inline void shutdown() {roi_sub_.shutdown();}

  friend class FacesDisplay;

};

/**
 * \class FacesDisplay
 *
 */
class FacesDisplay : public ImageDisplayBase
{

  Q_OBJECT
public:
  FacesDisplay();
  ~FacesDisplay() override;

  // Overrides from Display
  void onInitialize() override;
  void update(float wall_dt, float ros_dt) override;
  void reset() override;

public Q_SLOTS:
  virtual void updateNormalizeOptions();

protected:
  // overrides from Display
  void onEnable() override;
  void onDisable() override;

  /* This is called by incomingMessage(). */
  void processMessage(const sensor_msgs::Image::ConstPtr& msg) override;

  Ogre::SceneManager* img_scene_manager_;

  ROSImageTexture texture_;

  RenderPanel* render_panel_;

  //ros::NodeHandle nh_;
  ros::Subscriber faces_list_sub_;
  std::vector<std::string> ids_;
  ros::Subscriber face_roi_sub_; //Currently just one face detected
  cv_bridge::CvImagePtr cvBridge_;

  bool checkBbConsistency(const int& image_height,
                            const int& image_width,
                            const int& bb_x,
                            const int& bb_y,
                            const int& bb_height,
                            const int& bb_width) const;

private:
  Ogre::SceneNode* img_scene_node_;
  Ogre::Rectangle2D* screen_rect_;
  Ogre::MaterialPtr material_;

  BoolProperty* normalize_property_;
  FloatProperty* min_property_;
  FloatProperty* max_property_;
  IntProperty* median_buffer_size_property_;
  bool got_float_image_;
  std::map<std::string, BoundingBox> faces_; 

  void list_callback(const hri_msgs::IdsListConstPtr& msg);

  void bb_callback(const hri_msgs::RegionOfInterestStampedConstPtr& msg, const std::string& id);

  friend class BoundingBox;

};

} // namespace rviz

#endif