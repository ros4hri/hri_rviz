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

#include "hri_humans.hpp"

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
#include <hri_msgs/NormalizedPointOfInterest2D.h>
#include <hri_msgs/Skeleton2D.h>
#include <image_transport/image_transport.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include <rviz/ogre_helpers/compatibility.h>
#include <rviz/render_panel.h>
#include <rviz/validate_floats.h>
#include <sensor_msgs/image_encodings.h>
#include <stdlib.h>  // srand, rand

#include <boost/bind.hpp>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#define SKELETON_POINTS 18

#define JOINT_RADIUS 8

using namespace std;

cv::Scalar get_color_from_id(std::string id) {
  hash<string> hasher;
  size_t hash = hasher(id);
  srand(hash);
  cv::Mat3f hsv(cv::Vec3f(rand() % 360, 0.7, 0.8));
  cv::Mat3f bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  return cv::Scalar(bgr(0, 0) * 255);
}

int clip(int n, int lower, int upper){
  return std::max(lower, std::min(n, upper));
}

namespace rviz {
HumansDisplay::HumansDisplay() : ImageDisplayBase(), texture_() {
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
      "Show face RoIs", true, "If set to true, show faces bounding boxes.",
      this, SLOT(updateShowFaces()));

  show_facial_landmarks_property_ = new BoolProperty(
      "Show facial landmarks", true, "If set to true, show faces facial landmarks.",
      this, SLOT(updateShowFacialLandmarks()));

  show_bodies_property_ = new BoolProperty(
      "Show body RoIs", true, "If set to true, show bodies bounding boxes.",
      this, SLOT(updateShowBodies()));

  show_skeletons_property_ = new BoolProperty(
      "Show 2D Skeletons", true, "If set to true, show 2D skeletons.",
      this, SLOT(updateShowSkeletons()));

  show_faces_ = true;
  show_facial_landmarks_ = true;
  show_bodies_ = true;
  show_skeletons_ = true;
  got_float_image_ = false;
}

void HumansDisplay::onInitialize() {
  ImageDisplayBase::onInitialize();
  {
    static uint32_t count = 0;
    std::stringstream ss;
    ss << "HumansDisplay" << count++;
    img_scene_manager_ = Ogre::Root::getSingleton().createSceneManager(
        Ogre::ST_GENERIC, ss.str());
  }

  img_scene_node_ =
      img_scene_manager_->getRootSceneNode()->createChildSceneNode();

  {
    static int count = 0;
    std::stringstream ss;
    ss << "HumansDisplayObject" << count++;

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

HumansDisplay::~HumansDisplay() {
  if (initialized()) {
    delete render_panel_;
    delete screen_rect_;
    removeAndDestroyChildNode(img_scene_node_->getParentSceneNode(),
                              img_scene_node_);
  }
}

void HumansDisplay::onEnable() {
  ImageDisplayBase::subscribe();
  render_panel_->getRenderWindow()->setActive(true);
}

void HumansDisplay::onDisable() {
  render_panel_->getRenderWindow()->setActive(false);
  ImageDisplayBase::unsubscribe();
  reset();
}

void HumansDisplay::updateShowFaces() {
  show_faces_ = show_faces_property_->getBool();
}

void HumansDisplay::updateShowFacialLandmarks() {
  show_facial_landmarks_ = show_facial_landmarks_property_->getBool();
}

void HumansDisplay::updateShowBodies() {
  show_bodies_ = show_bodies_property_->getBool();
}

void HumansDisplay::updateShowSkeletons() {
  show_skeletons_ = show_skeletons_property_->getBool();
}

void HumansDisplay::updateNormalizeOptions() {
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

void HumansDisplay::update(float wall_dt, float ros_dt) {
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

void HumansDisplay::reset() {
  ImageDisplayBase::reset();
  texture_.clear();
  render_panel_->getCamera()->setPosition(
      Ogre::Vector3(999999, 999999, 999999));
}

void HumansDisplay::drawSkeleton(std::string id, int width, int height, std::vector<hri_msgs::NormalizedPointOfInterest2D>& skeleton){
  /* Body chains:
     1 - 2 - 8 - 11 - 5 ==> Upper body chain
     2 - 3 - 4 ==> Right arm chain
     5 - 6 - 7 ==> Left arm chain
     8 - 9 - 10 ==> Right leg chain
     11 - 12 - 13 ==> Left leg chain
  */

  if (skeleton.size() == SKELETON_POINTS) {

    cv::Scalar skeletonColor = get_color_from_id(id);

    int neckX = clip((int)(skeleton[hri_msgs::Skeleton2D::NECK].x*width), 0, width);
    int neckY = clip((int)(skeleton[hri_msgs::Skeleton2D::NECK].y*height), 0, height);

    cv::circle(cvBridge_->image, cv::Point(neckX, neckY), JOINT_RADIUS, skeletonColor, cv::FILLED);

    int rightShoulderX = clip((int)(skeleton[hri_msgs::Skeleton2D::RIGHT_SHOULDER].x*width), 0, width);
    int rightShoulderY = clip((int)(skeleton[hri_msgs::Skeleton2D::RIGHT_SHOULDER].y*height), 0, height);

    cv::circle(cvBridge_->image, cv::Point(rightShoulderX, rightShoulderY), JOINT_RADIUS, skeletonColor, cv::FILLED);

    int rightHipX = clip((int)(skeleton[hri_msgs::Skeleton2D::RIGHT_HIP].x*width), 0, width);
    int rightHipY = clip((int)(skeleton[hri_msgs::Skeleton2D::RIGHT_HIP].y*height), 0, height);

    cv::circle(cvBridge_->image, cv::Point(rightHipX, rightHipY), JOINT_RADIUS, skeletonColor, cv::FILLED);    

    int leftHipX = clip((int)(skeleton[hri_msgs::Skeleton2D::LEFT_HIP].x*width), 0, width);
    int leftHipY = clip((int)(skeleton[hri_msgs::Skeleton2D::LEFT_HIP].y*height), 0, height);

    cv::circle(cvBridge_->image, cv::Point(leftHipX, leftHipY), JOINT_RADIUS, skeletonColor, cv::FILLED);

    int leftShoulderX = clip((int)(skeleton[hri_msgs::Skeleton2D::LEFT_SHOULDER].x*width), 0, width);
    int leftShoulderY = clip((int)(skeleton[hri_msgs::Skeleton2D::LEFT_SHOULDER].y*height), 0, height);

    cv::circle(cvBridge_->image, cv::Point(leftShoulderX, leftShoulderY), JOINT_RADIUS, skeletonColor, cv::FILLED);

    int rightElbowX = clip((int)(skeleton[hri_msgs::Skeleton2D::RIGHT_ELBOW].x*width), 0, width);
    int rightElbowY = clip((int)(skeleton[hri_msgs::Skeleton2D::RIGHT_ELBOW].y*height), 0, height);

    cv::circle(cvBridge_->image, cv::Point(rightElbowX, rightElbowY), JOINT_RADIUS, skeletonColor, cv::FILLED);

    int rightWristX = clip((int)(skeleton[hri_msgs::Skeleton2D::RIGHT_WRIST].x*width), 0, width);
    int rightWristY = clip((int)(skeleton[hri_msgs::Skeleton2D::RIGHT_WRIST].y*height), 0, height);

    cv::circle(cvBridge_->image, cv::Point(rightWristX, rightWristY), JOINT_RADIUS, skeletonColor, cv::FILLED);

    int leftElbowX = clip((int)(skeleton[hri_msgs::Skeleton2D::LEFT_ELBOW].x*width), 0, width);
    int leftElbowY = clip((int)(skeleton[hri_msgs::Skeleton2D::LEFT_ELBOW].y*height), 0, height);

    cv::circle(cvBridge_->image, cv::Point(leftElbowX, leftElbowY), JOINT_RADIUS, skeletonColor, cv::FILLED);

    int leftWristX = clip((int)(skeleton[hri_msgs::Skeleton2D::LEFT_WRIST].x*width), 0, width);
    int leftWristY = clip((int)(skeleton[hri_msgs::Skeleton2D::LEFT_WRIST].y*height), 0, height);

    cv::circle(cvBridge_->image, cv::Point(leftWristX, leftWristY), JOINT_RADIUS, skeletonColor, cv::FILLED);

    int rightKneeX = clip((int)(skeleton[hri_msgs::Skeleton2D::RIGHT_KNEE].x*width), 0, width);
    int rightKneeY = clip((int)(skeleton[hri_msgs::Skeleton2D::RIGHT_KNEE].y*height), 0, height);

    cv::circle(cvBridge_->image, cv::Point(rightKneeX, rightKneeY), JOINT_RADIUS, skeletonColor, cv::FILLED);

    int rightAnkleX = clip((int)(skeleton[hri_msgs::Skeleton2D::RIGHT_ANKLE].x*width), 0, width);
    int rightAnkleY = clip((int)(skeleton[hri_msgs::Skeleton2D::RIGHT_ANKLE].y*height), 0, height);

    cv::circle(cvBridge_->image, cv::Point(rightAnkleX, rightAnkleY), JOINT_RADIUS, skeletonColor, cv::FILLED);

    int leftKneeX = clip((int)(skeleton[hri_msgs::Skeleton2D::LEFT_KNEE].x*width), 0, width);
    int leftKneeY = clip((int)(skeleton[hri_msgs::Skeleton2D::LEFT_KNEE].y*height), 0, height);

    cv::circle(cvBridge_->image, cv::Point(leftKneeX, leftKneeY), JOINT_RADIUS, skeletonColor, cv::FILLED);

    int leftAnkleX = clip((int)(skeleton[hri_msgs::Skeleton2D::LEFT_ANKLE].x*width), 0, width);
    int leftAnkleY = clip((int)(skeleton[hri_msgs::Skeleton2D::LEFT_ANKLE].y*height), 0, height);

    cv::circle(cvBridge_->image, cv::Point(leftAnkleX, leftAnkleY), JOINT_RADIUS, skeletonColor, cv::FILLED);

    // Upper body
    cv::line(cvBridge_->image, cv::Point(neckX, neckY), cv::Point(rightShoulderX, rightShoulderY), skeletonColor, 5, cv::FILLED);
    cv::line(cvBridge_->image, cv::Point(rightHipX, rightHipY), cv::Point(rightShoulderX, rightShoulderY), skeletonColor, 5, cv::FILLED);
    cv::line(cvBridge_->image, cv::Point(neckX, neckY), cv::Point(leftShoulderX, leftShoulderY), skeletonColor, 5, cv::FILLED);
    cv::line(cvBridge_->image, cv::Point(leftHipX, leftHipY), cv::Point(leftShoulderX, leftShoulderY), skeletonColor, 5, cv::FILLED);
    cv::line(cvBridge_->image, cv::Point(leftHipX, leftHipY), cv::Point(rightHipX, rightHipY), skeletonColor, 5, cv::FILLED);

    // Right arm
    cv::line(cvBridge_->image, cv::Point(rightShoulderX, rightShoulderY), cv::Point(rightElbowX, rightElbowY), skeletonColor, 5, cv::FILLED);
    cv::line(cvBridge_->image, cv::Point(rightElbowX, rightElbowY), cv::Point(rightWristX, rightWristY), skeletonColor, 5, cv::FILLED);

    // Left arm
    cv::line(cvBridge_->image, cv::Point(leftShoulderX, leftShoulderY), cv::Point(leftElbowX, leftElbowY), skeletonColor, 5, cv::FILLED);
    cv::line(cvBridge_->image, cv::Point(leftElbowX, leftElbowY), cv::Point(leftWristX, leftWristY), skeletonColor, 5, cv::FILLED);
    
    // Right Leg
    cv::line(cvBridge_->image, cv::Point(rightHipX, rightHipY), cv::Point(rightKneeX, rightKneeY), skeletonColor, 5, cv::FILLED);
    cv::line(cvBridge_->image, cv::Point(rightKneeX, rightKneeY), cv::Point(rightAnkleX, rightAnkleY), skeletonColor, 5, cv::FILLED);

    // Left leg
    cv::line(cvBridge_->image, cv::Point(leftHipX, leftHipY), cv::Point(leftKneeX, leftKneeY), skeletonColor, 5, cv::FILLED);
    cv::line(cvBridge_->image, cv::Point(leftKneeX, leftKneeY), cv::Point(leftAnkleX, leftAnkleY), skeletonColor, 5, cv::FILLED);

  }
}

void HumansDisplay::processMessage(const sensor_msgs::Image::ConstPtr& msg) {
  bool got_float_image =
      msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1 ||
      msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
      msg->encoding == sensor_msgs::image_encodings::TYPE_16SC1 ||
      msg->encoding == sensor_msgs::image_encodings::MONO16;

  if (got_float_image != got_float_image_) {
    got_float_image_ = got_float_image;
    updateNormalizeOptions();
  }

  if (!show_faces_ && !show_bodies_ && !show_skeletons_) {
    texture_.addMessage(msg);
    return;
  }

  cvBridge_ = cv_bridge::toCvCopy(msg);

  if (show_faces_ || show_facial_landmarks_) {
    auto faces = hri_listener.getFaces();
    for (auto const& face : faces) {
      if (auto face_ptr =
              face.second.lock()) {  // ensure the face is still here
        if(show_faces_){
          auto roi = face_ptr->roi();
          cv::Point roi_tl(static_cast<int>(roi.xmin * msg->width),
                           static_cast<int>(roi.ymin * msg->height));
          cv::Point roi_br(static_cast<int>(roi.xmax * msg->width),
                           static_cast<int>(roi.ymax * msg->height));
          cv::rectangle(cvBridge_->image, roi_tl, roi_br, get_color_from_id(face.first), 5);
        }
        if(show_facial_landmarks_){
          auto landmarks = *(face_ptr->facialLandmarks()); // boost::optional
          for(auto landmark : landmarks){
            if(landmark.x > 0 || landmark.y > 0)
              cv::circle(cvBridge_->image, cv::Point(landmark.x, landmark.y), 5, get_color_from_id(face.first), cv::FILLED);
          }
        }
      }
    }
  }

  if (show_bodies_ || show_skeletons_) {
    auto bodies = hri_listener.getBodies();
    for (auto const& body : bodies) {
      if (auto body_ptr =
              body.second.lock()) {  // ensure the body is still here
        if (show_bodies_){
          auto roi = body_ptr->roi();
          cv::Point roi_tl(static_cast<int>(roi.xmin * msg->width),
                           static_cast<int>(roi.ymin * msg->height));
          cv::Point roi_br(static_cast<int>(roi.xmax * msg->width),
                           static_cast<int>(roi.ymax * msg->height));
          cv::rectangle(cvBridge_->image, roi_tl, roi_br, get_color_from_id(body.first), 5);
        }
        if (show_skeletons_){
          auto skeleton = body_ptr->skeleton();
          drawSkeleton(body.first, msg->width, msg->height, skeleton);
        }
      }
    }
  }

  texture_.addMessage(cvBridge_->toImageMsg());
}

}  // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz::HumansDisplay, rviz::Display)
