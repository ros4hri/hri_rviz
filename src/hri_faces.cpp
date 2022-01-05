/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#include "hri_faces.hpp"

#include <OGRE/OgreCamera.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreRectangle2D.h>
#include <OGRE/OgreRenderSystem.h>
#include <OGRE/OgreRenderWindow.h>
#include <OGRE/OgreRoot.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreTechnique.h>
#include <OGRE/OgreTextureManager.h>
#include <OGRE/OgreViewport.h>
#include <cv_bridge/cv_bridge.h>
#include <hri_msgs/IdsList.h>
#include <hri_msgs/RegionOfInterestStamped.h>
#include <image_transport/image_transport.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/compatibility.h>
#include <rviz/render_panel.h>
#include <rviz/validate_floats.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <sensor_msgs/image_encodings.h>

#include <boost/bind.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <vector>

namespace rviz {

BoundingBox::BoundingBox(const std::string& ns, const std::string& id,
                         ros::NodeHandle& nh, int R, int G, int B,
                         FacesDisplay* obj)
    : R_(R), G_(G), B_(B) {
  std::string topic = ns + id + "/roi";
  roi_sub_ = nh.subscribe<hri_msgs::RegionOfInterestStamped>(
      topic, 1,
      std::bind(&FacesDisplay::bb_callback, obj, std::placeholders::_1, id));
  width = 0;
}

FacesDisplay::FacesDisplay() : ImageDisplayBase(), texture_() {
  normalize_property_ =
      new BoolProperty("Normalize Range", true,
                       "If set to true, will try to estimate the range of "
                       "possible values from the received images.",
                       this, SLOT(updateNormalizeOptions()));

  min_property_ = new FloatProperty("Min Value", 0.0,
                                    "Value which will be displayed as black.",
                                    this, SLOT(updateNormalizeOptions()));

  max_property_ = new FloatProperty("Max Value", 1.0,
                                    "Value which will be displayed as white.",
                                    this, SLOT(updateNormalizeOptions()));

  median_buffer_size_property_ = new IntProperty(
      "Median window", 5,
      "Window size for median filter used for computin min/max.", this,
      SLOT(updateNormalizeOptions()));

  show_faces_property_ = new BoolProperty(
      "Faces", true, "If set to true, show faces bounding boxes.", this,
      SLOT(updateShowFaces()));

  show_bodies_property_ = new BoolProperty(
      "Bodies", false, "If set to true, show bodies bounding boxes.", this,
      SLOT(updateShowBodies()));

  show_faces_ = true;
  show_bodies_ = false;
  got_float_image_ = false;
}

void FacesDisplay::onInitialize() {
  ImageDisplayBase::onInitialize();
  {
    static uint32_t count = 0;
    std::stringstream ss;
    ss << "FacesDisplay" << count++;
    img_scene_manager_ = Ogre::Root::getSingleton().createSceneManager(
        Ogre::ST_GENERIC, ss.str());

    faces_list_sub_ = update_nh_.subscribe("/humans/faces/tracked", 1,
                                           &FacesDisplay::list_callback, this);
    /*bodies_list_sub_ =
      update_nh_.subscribe("/humans/bodies/tracked",
                           1,
                           &FacesDisplay::bodyListCallback,
                           this);*/
  }

  img_scene_node_ =
      img_scene_manager_->getRootSceneNode()->createChildSceneNode();

  {
    static int count = 0;
    std::stringstream ss;
    ss << "FacesDisplayObject" << count++;

    screen_rect_ = new Ogre::Rectangle2D(true);
    screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
    screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

    ss << "Material";
    material_ = Ogre::MaterialManager::getSingleton().create(
        ss.str(), Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
    material_->setSceneBlending(Ogre::SBT_REPLACE);
    material_->setDepthWriteEnabled(false);
    material_->setReceiveShadows(false);
    material_->setDepthCheckEnabled(false);

    material_->getTechnique(0)->setLightingEnabled(false);
    Ogre::TextureUnitState* tu =
        material_->getTechnique(0)->getPass(0)->createTextureUnitState();
    tu->setTextureName(texture_.getTexture()->getName());
    tu->setTextureFiltering(Ogre::TFO_NONE);

    material_->setCullingMode(Ogre::CULL_NONE);
    Ogre::AxisAlignedBox aabInf;
    aabInf.setInfinite();
    screen_rect_->setBoundingBox(aabInf);
    setMaterial(*screen_rect_, material_);
    img_scene_node_->attachObject(screen_rect_);
  }

  render_panel_ = new RenderPanel();
  render_panel_->getRenderWindow()->setAutoUpdated(false);
  render_panel_->getRenderWindow()->setActive(false);

  render_panel_->resize(640, 480);
  render_panel_->initialize(img_scene_manager_, context_);

  setAssociatedWidget(render_panel_);

  render_panel_->setAutoRender(false);
  render_panel_->setOverlaysEnabled(false);
  render_panel_->getCamera()->setNearClipDistance(0.01f);

  updateNormalizeOptions();
}

FacesDisplay::~FacesDisplay() {
  if (initialized()) {
    delete render_panel_;
    delete screen_rect_;
    removeAndDestroyChildNode(img_scene_node_->getParentSceneNode(),
                              img_scene_node_);
  }
}

void FacesDisplay::onEnable() {
  ImageDisplayBase::subscribe();
  render_panel_->getRenderWindow()->setActive(true);
}

void FacesDisplay::onDisable() {
  render_panel_->getRenderWindow()->setActive(false);
  ImageDisplayBase::unsubscribe();
  reset();
}

void FacesDisplay::updateShowFaces() {
  show_faces_ = show_faces_property_->getBool();
}

void FacesDisplay::updateShowBodies() {
  show_bodies_ = show_bodies_property_->getBool();
}

void FacesDisplay::updateNormalizeOptions() {
  if (got_float_image_) {
    bool normalize = normalize_property_->getBool();

    normalize_property_->setHidden(false);
    min_property_->setHidden(normalize);
    max_property_->setHidden(normalize);
    median_buffer_size_property_->setHidden(!normalize);

    texture_.setNormalizeFloatImage(normalize, min_property_->getFloat(),
                                    max_property_->getFloat());
    texture_.setMedianFrames(median_buffer_size_property_->getInt());
  } else {
    normalize_property_->setHidden(true);
    min_property_->setHidden(true);
    max_property_->setHidden(true);
    median_buffer_size_property_->setHidden(true);
  }
}

void FacesDisplay::update(float wall_dt, float ros_dt) {
  Q_UNUSED(wall_dt)
  Q_UNUSED(ros_dt)
  try {
    texture_.update();

    // make sure the aspect ratio of the image is preserved
    float win_width = render_panel_->width();
    float win_height = render_panel_->height();

    float img_width = texture_.getWidth();
    float img_height = texture_.getHeight();

    if (img_width != 0 && img_height != 0 && win_width != 0 &&
        win_height != 0) {
      float img_aspect = img_width / img_height;
      float win_aspect = win_width / win_height;

      if (img_aspect > win_aspect) {
        screen_rect_->setCorners(-1.0f, 1.0f * win_aspect / img_aspect, 1.0f,
                                 -1.0f * win_aspect / img_aspect, false);
      } else {
        screen_rect_->setCorners(-1.0f * img_aspect / win_aspect, 1.0f,
                                 1.0f * img_aspect / win_aspect, -1.0f, false);
      }
    }

    render_panel_->getRenderWindow()->update();
  } catch (UnsupportedImageEncoding& e) {
    setStatus(StatusProperty::Error, "Image", e.what());
  }
}

void FacesDisplay::reset() {
  ImageDisplayBase::reset();
  texture_.clear();
  render_panel_->getCamera()->setPosition(
      Ogre::Vector3(999999, 999999, 999999));
}

void FacesDisplay::processMessage(const sensor_msgs::Image::ConstPtr& msg) {
  bool got_float_image =
      msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
      msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
      msg->encoding == sensor_msgs::image_encodings::TYPE_16SC1 ||
      msg->encoding == sensor_msgs::image_encodings::MONO16;

  if (got_float_image != got_float_image_) {
    got_float_image_ = got_float_image;
    updateNormalizeOptions();
  }

  if ((!show_faces_ && !show_bodies_) ||
      ((faces_.size() == 0) && (bodies_.size() == 0))) {
    texture_.addMessage(msg);
    return;
  }

  cvBridge_ = cv_bridge::toCvCopy(msg);

  if (faces_.size() > 0 && show_faces_) {
    std::map<std::string, BoundingBox>::iterator it;
    for (it = faces_.begin(); it != faces_.end(); ++it) {
      if (it->second.bbInitialized()) {
        cv::rectangle(cvBridge_->image, it->second.getRect(),
                      it->second.getRGB(), 5);
      }
    }
  }

  if (bodies_.size() > 0 && show_bodies_) {
    std::map<std::string, BoundingBox>::iterator it;
    for (it = bodies_.begin(); it != bodies_.end(); ++it) {
      if (it->second.bbInitialized()) {
        cv::rectangle(cvBridge_->image, it->second.getRect(),
                      it->second.getRGB(), 5);
      }
    }
  }

  texture_.addMessage(cvBridge_->toImageMsg());
}

void FacesDisplay::list_callback(const hri_msgs::IdsListConstPtr& msg) {
  if (ros::ok()) {
    ids_ = msg->ids;

    // Check for faces that are no more in the list
    // Remove them from the map

    std::map<std::string, BoundingBox>::iterator itF;
    for (itF = faces_.begin(); itF != faces_.end();) {
      if (std::find(ids_.begin(), ids_.end(), itF->first) == ids_.end()) {
        itF->second.shutdown();
        faces_.erase((itF++)->first);
      } else
        ++itF;
    }

    // Check for new faces
    // Create a bounding box message and insert it in the map

    std::vector<std::string>::iterator itS;
    for (itS = ids_.begin(); itS != ids_.end(); ++itS) {
      if (faces_.find(*itS) == faces_.end()) {
        std::string id = *itS;
        faces_.insert(std::pair<std::string, BoundingBox>(
            id, BoundingBox("/humans/faces/", id, update_nh_, std::rand() % 256,
                            std::rand() % 256, std::rand() % 256, this)));
      }
    }
  }
}

void FacesDisplay::bodyListCallback(const hri_msgs::IdsListConstPtr& msg) {
  ROS_WARN("Body list processing");

  if (ros::ok()) {
    ROS_WARN("Inside");
    body_ids_ = msg->ids;

    std::cout << "Old number of bodies: " << bodies_.size()
              << "\t New number of bodies: " << body_ids_.size() << std::endl;

    // Check for faces that are no more in the list
    // Remove them from the map

    std::map<std::string, BoundingBox>::iterator itF;
    for (itF = bodies_.begin(); itF != bodies_.end();) {
      std::cout << itF->first << std::endl;
      ROS_WARN("Inside 1");
      // std::cout<<itF->first<<std::endl;
      ROS_WARN("Inside 1.5");
      if (std::find(body_ids_.begin(), body_ids_.end(), itF->first) ==
          body_ids_.end()) {
        ROS_WARN("Deleting Body: start");
        itF->second.shutdown();
        ROS_WARN("Deleting Body: mid");
        bodies_.erase((itF++)->first);
        ROS_WARN("Deleting Body: end");
      } else
        ++itF;
    }
    ROS_WARN("Outside 1");

    // Check for new faces
    // Create a bounding box message and insert it in the map

    std::vector<std::string>::iterator itS;
    for (itS = body_ids_.begin(); itS != body_ids_.end(); ++itS) {
      ROS_WARN("Inside 2");
      if (bodies_.find(*itS) == bodies_.end()) {
        ROS_WARN("Inserting new body");
        std::string id = *itS;
        bodies_.insert(std::pair<std::string, BoundingBox>(
            id,
            BoundingBox("/humans/bodies/", id, update_nh_, std::rand() % 256,
                        std::rand() % 256, std::rand() % 256, this)));
      }
    }
    ROS_WARN("outside 2");
  }
  ROS_WARN("Out");
}

void FacesDisplay::bb_callback(
    const hri_msgs::RegionOfInterestStampedConstPtr& msg,
    const std::string& id) {
  BoundingBox& face = faces_.find(id)->second;

  face.x = msg->roi.x_offset;
  face.y = msg->roi.y_offset;
  face.width = msg->roi.width;
  face.height = msg->roi.height;
}

bool FacesDisplay::checkBbConsistency(const int& image_height,
                                      const int& image_width, const int& bb_x,
                                      const int& bb_y, const int& bb_height,
                                      const int& bb_width) const {
  return (bb_x >= 0) && (bb_y >= 0) && (bb_height > 0) && (bb_width > 0) &&
         ((bb_y + bb_height) < image_height) &&
         ((bb_x + bb_width) < image_width);
}

}  // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::FacesDisplay, rviz::Display)