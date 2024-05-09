// Copyright 2012, Willow Garage, Inc.
// Copyright 2024, PAL Robotics S.L.
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

#include "hri_rviz/hri_humans.hpp"

#include <OgreCamera.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreRectangle2D.h>
#include <OgreRenderSystem.h>
#include <OgreRenderWindow.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreViewport.h>

#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/validate_floats.hpp>
#include <rviz_common/uniform_string_stream.hpp>
#include <rviz_rendering/material_manager.hpp>
#include <rviz_rendering/render_window.hpp>

#include <rviz_default_plugins/displays/image/ros_image_texture.hpp>
#include <rviz_default_plugins/displays/image/image_display.hpp>
#include <rviz_default_plugins/displays/image/ros_image_texture_iface.hpp>

#include <sensor_msgs/image_encodings.hpp>
#include <hri_msgs/msg/skeleton2_d.hpp>

#include <opencv2/opencv.hpp>

#include <stdlib.h>
#include <sstream>

using namespace std;

constexpr int SKELETON_POINTS = 18;
constexpr int JOINT_RADIUS = 8;

cv::Scalar get_color_from_id(std::string id)
{
  hash<string> hasher;
  size_t hash = hasher(id);
  srand(hash);
  cv::Mat3f hsv(cv::Vec3f(rand() % 360, 0.7, 0.8));
  cv::Mat3f bgr;
  cv::cvtColor(hsv, bgr, cv::COLOR_HSV2BGR);
  return cv::Scalar(bgr(0, 0) * 255);
}

int clip(int n, int lower, int upper)
{
  return std::max(lower, std::min(n, upper));
}

namespace rviz_hri_plugins
{

HumansDisplay::HumansDisplay()
: HumansDisplay(std::make_unique<rviz_default_plugins::displays::ROSImageTexture>()) {}

HumansDisplay::HumansDisplay(
  std::unique_ptr<rviz_default_plugins::displays::ROSImageTextureIface> texture)
: texture_(std::move(texture))
{
  hri_executor_ = rclcpp::executors::MultiThreadedExecutor::make_shared();
  hri_node_ = rclcpp::Node::make_shared("hri_node_hri_humans");
  hri_executor_->add_node(hri_node_);
  hri_listener_ = hri::HRIListener::create(hri_node_);

  normalize_property_ =
    new BoolProperty(
    "Normalize Range", true,
    "If set to true, will try to estimate the range of "
    "possible values from the received images.",
    this, SLOT(updateNormalizeOptions()));

  min_property_ = new rviz_common::properties::FloatProperty(
    "Min Value", 0.0,
    "Value which will be displayed as black.",
    this, SLOT(updateNormalizeOptions()));

  max_property_ = new rviz_common::properties::FloatProperty(
    "Max Value", 1.0,
    "Value which will be displayed as white.",
    this, SLOT(updateNormalizeOptions()));

  median_buffer_size_property_ = new rviz_common::properties::IntProperty(
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

void HumansDisplay::onInitialize()
{
  ITDClass::onInitialize();

  updateNormalizeOptions();
  setupScreenRectangle();

  setupRenderPanel();

  render_panel_->getRenderWindow()->setupSceneAfterInit(
    [this](Ogre::SceneNode * scene_node) {
      scene_node->attachObject(screen_rect_.get());
    });
}

HumansDisplay::~HumansDisplay() = default;

void HumansDisplay::onEnable()
{
  ITDClass::subscribe();
}

void HumansDisplay::onDisable()
{
  ITDClass::unsubscribe();
  clear();
}

void HumansDisplay::updateShowFaces()
{
  show_faces_ = show_faces_property_->getBool();
}

void HumansDisplay::updateShowFacialLandmarks()
{
  show_facial_landmarks_ = show_facial_landmarks_property_->getBool();
}

void HumansDisplay::updateShowBodies()
{
  show_bodies_ = show_bodies_property_->getBool();
}

void HumansDisplay::updateShowSkeletons()
{
  show_skeletons_ = show_skeletons_property_->getBool();
}

void HumansDisplay::updateNormalizeOptions()
{
  if (got_float_image_) {
    bool normalize = normalize_property_->getBool();

    normalize_property_->setHidden(false);
    min_property_->setHidden(normalize);
    max_property_->setHidden(normalize);
    median_buffer_size_property_->setHidden(!normalize);

    texture_->setNormalizeFloatImage(
      normalize, min_property_->getFloat(),
      max_property_->getFloat());
    texture_->setMedianFrames(median_buffer_size_property_->getInt());
  } else {
    normalize_property_->setHidden(true);
    min_property_->setHidden(true);
    max_property_->setHidden(true);
    median_buffer_size_property_->setHidden(true);
  }
}

void HumansDisplay::clear()
{
  texture_->clear();
}

void HumansDisplay::update(float wall_dt, float ros_dt)
{
  (void) wall_dt;
  (void) ros_dt;
  try {
    texture_->update();

    // make sure the aspect ratio of the image is preserved
    auto win_width = render_panel_->width();
    auto win_height = render_panel_->height();

    auto img_width = texture_->getWidth();
    auto img_height = texture_->getHeight();

    if (img_width != 0 && img_height != 0 && win_width != 0 &&
      win_height != 0)
    {

      float img_aspect = static_cast<float>(img_width) / static_cast<float>(img_height);
      float win_aspect = static_cast<float>(win_width) / static_cast<float>(win_height);

      if (img_aspect > win_aspect) {
        screen_rect_->setCorners(
          -1.0f, 1.0f * win_aspect / img_aspect, 1.0f,
          -1.0f * win_aspect / img_aspect, false);
      } else {
        screen_rect_->setCorners(
          -1.0f * img_aspect / win_aspect, 1.0f,
          1.0f * img_aspect / win_aspect, -1.0f, false);
      }
    }
  } catch (rviz_default_plugins::displays::UnsupportedImageEncoding & e) {
    setStatus(rviz_common::properties::StatusProperty::Error, "Image", e.what());
  }
}

void HumansDisplay::reset()
{
  ITDClass::reset();
  clear();
}

void HumansDisplay::drawSkeleton(
  std::string id, int width, int height,
  std::map<hri::SkeletalKeypoint, hri::PointOfInterest> & skeleton)
{
  /* Body chains:
     1 - 2 - 8 - 11 - 5 ==> Upper body chain
     2 - 3 - 4 ==> Right arm chain
     5 - 6 - 7 ==> Left arm chain
     8 - 9 - 10 ==> Right leg chain
     11 - 12 - 13 ==> Left leg chain
  */

  if (skeleton.size() == SKELETON_POINTS) {

    cv::Scalar skeletonColor = get_color_from_id(id);

    hri::PointOfInterest neckPoI = skeleton[hri::SkeletalKeypoint::kNeck];
    int neckX = clip((int)(neckPoI.x * width), 0, width);
    int neckY = clip((int)(neckPoI.y * height), 0, height);

    cv::circle(cvBridge_->image, cv::Point(neckX, neckY), JOINT_RADIUS, skeletonColor, cv::FILLED);

    hri::PointOfInterest rightShoulderPoI = skeleton[hri::SkeletalKeypoint::kRightShoulder];
    int rightShoulderX = clip((int)(rightShoulderPoI.x * width), 0, width);
    int rightShoulderY = clip((int)(rightShoulderPoI.y * height), 0, height);

    cv::circle(
      cvBridge_->image, cv::Point(
        rightShoulderX,
        rightShoulderY), JOINT_RADIUS, skeletonColor,
      cv::FILLED);

    hri::PointOfInterest rightHipPoI = skeleton[hri::SkeletalKeypoint::kRightHip];
    int rightHipX = clip((int)(rightHipPoI.x * width), 0, width);
    int rightHipY = clip((int)(rightHipPoI.y * height), 0, height);

    cv::circle(
      cvBridge_->image, cv::Point(
        rightHipX,
        rightHipY), JOINT_RADIUS, skeletonColor, cv::FILLED);

    hri::PointOfInterest leftHipPoI = skeleton[hri::SkeletalKeypoint::kLeftHip];
    int leftHipX = clip((int)(leftHipPoI.x * width), 0, width);
    int leftHipY = clip((int)(leftHipPoI.y * height), 0, height);

    cv::circle(
      cvBridge_->image, cv::Point(
        leftHipX,
        leftHipY), JOINT_RADIUS, skeletonColor, cv::FILLED);

    hri::PointOfInterest leftShoulderPoI = skeleton[hri::SkeletalKeypoint::kLeftShoulder];
    int leftShoulderX = clip((int)(leftShoulderPoI.x * width), 0, width);
    int leftShoulderY = clip((int)(leftShoulderPoI.y * height), 0, height);

    cv::circle(
      cvBridge_->image, cv::Point(
        leftShoulderX,
        leftShoulderY), JOINT_RADIUS, skeletonColor, cv::FILLED);

    hri::PointOfInterest rightElbowPoI = skeleton[hri::SkeletalKeypoint::kRightElbow];
    int rightElbowX = clip((int)(rightElbowPoI.x * width), 0, width);
    int rightElbowY = clip((int)(rightElbowPoI.y * height), 0, height);

    cv::circle(
      cvBridge_->image, cv::Point(
        rightElbowX,
        rightElbowY), JOINT_RADIUS, skeletonColor, cv::FILLED);

    hri::PointOfInterest rightWristPoI = skeleton[hri::SkeletalKeypoint::kRightWrist];
    int rightWristX = clip((int)(rightWristPoI.x * width), 0, width);
    int rightWristY = clip((int)(rightWristPoI.y * height), 0, height);

    cv::circle(
      cvBridge_->image, cv::Point(
        rightWristX,
        rightWristY), JOINT_RADIUS, skeletonColor, cv::FILLED);

    hri::PointOfInterest leftElbowPoI = skeleton[hri::SkeletalKeypoint::kLeftElbow];
    int leftElbowX = clip((int)(leftElbowPoI.x * width), 0, width);
    int leftElbowY = clip((int)(leftElbowPoI.y * height), 0, height);

    cv::circle(
      cvBridge_->image, cv::Point(
        leftElbowX,
        leftElbowY), JOINT_RADIUS, skeletonColor, cv::FILLED);

    hri::PointOfInterest leftWristPoI = skeleton[hri::SkeletalKeypoint::kLeftWrist];
    int leftWristX = clip((int)(leftWristPoI.x * width), 0, width);
    int leftWristY = clip((int)(leftWristPoI.y * height), 0, height);

    cv::circle(
      cvBridge_->image, cv::Point(
        leftWristX,
        leftWristY), JOINT_RADIUS, skeletonColor, cv::FILLED);

    hri::PointOfInterest rightKneePoI = skeleton[hri::SkeletalKeypoint::kRightKnee];
    int rightKneeX = clip((int)(rightKneePoI.x * width), 0, width);
    int rightKneeY = clip((int)(rightKneePoI.y * height), 0, height);

    cv::circle(
      cvBridge_->image, cv::Point(
        rightKneeX,
        rightKneeY), JOINT_RADIUS, skeletonColor, cv::FILLED);

    hri::PointOfInterest rightAnklePoI = skeleton[hri::SkeletalKeypoint::kRightAnkle];
    int rightAnkleX = clip((int)(rightAnklePoI.x * width), 0, width);
    int rightAnkleY = clip((int)(rightAnklePoI.y * height), 0, height);

    cv::circle(
      cvBridge_->image, cv::Point(
        rightAnkleX,
        rightAnkleY), JOINT_RADIUS, skeletonColor, cv::FILLED);

    hri::PointOfInterest leftKneePoI = skeleton[hri::SkeletalKeypoint::kLeftKnee];
    int leftKneeX = clip((int)(leftKneePoI.x * width), 0, width);
    int leftKneeY = clip((int)(leftKneePoI.y * height), 0, height);

    cv::circle(
      cvBridge_->image, cv::Point(
        leftKneeX,
        leftKneeY), JOINT_RADIUS, skeletonColor, cv::FILLED);

    hri::PointOfInterest leftAnklePoI = skeleton[hri::SkeletalKeypoint::kLeftAnkle];
    int leftAnkleX = clip((int)(leftAnklePoI.x * width), 0, width);
    int leftAnkleY = clip((int)(leftAnklePoI.y * height), 0, height);

    cv::circle(
      cvBridge_->image, cv::Point(
        leftAnkleX,
        leftAnkleY), JOINT_RADIUS, skeletonColor, cv::FILLED);

    // Upper body
    cv::line(
      cvBridge_->image, cv::Point(neckX, neckY), cv::Point(
        rightShoulderX,
        rightShoulderY), skeletonColor, 5,
      cv::FILLED);
    cv::line(
      cvBridge_->image, cv::Point(rightHipX, rightHipY),
      cv::Point(rightShoulderX, rightShoulderY), skeletonColor, 5, cv::FILLED);
    cv::line(
      cvBridge_->image, cv::Point(neckX, neckY), cv::Point(
        leftShoulderX,
        leftShoulderY), skeletonColor, 5,
      cv::FILLED);
    cv::line(
      cvBridge_->image, cv::Point(leftHipX, leftHipY), cv::Point(
        leftShoulderX,
        leftShoulderY), skeletonColor, 5,
      cv::FILLED);
    cv::line(
      cvBridge_->image, cv::Point(leftHipX, leftHipY), cv::Point(
        rightHipX,
        rightHipY), skeletonColor, 5,
      cv::FILLED);

    // Right arm
    cv::line(
      cvBridge_->image, cv::Point(rightShoulderX, rightShoulderY),
      cv::Point(rightElbowX, rightElbowY), skeletonColor, 5, cv::FILLED);
    cv::line(
      cvBridge_->image, cv::Point(rightElbowX, rightElbowY),
      cv::Point(rightWristX, rightWristY), skeletonColor, 5, cv::FILLED);

    // Left arm
    cv::line(
      cvBridge_->image, cv::Point(leftShoulderX, leftShoulderY),
      cv::Point(leftElbowX, leftElbowY), skeletonColor, 5, cv::FILLED);
    cv::line(
      cvBridge_->image, cv::Point(leftElbowX, leftElbowY), cv::Point(
        leftWristX,
        leftWristY), skeletonColor, 5,
      cv::FILLED);

    // Right Leg
    cv::line(
      cvBridge_->image, cv::Point(rightHipX, rightHipY), cv::Point(
        rightKneeX,
        rightKneeY), skeletonColor, 5,
      cv::FILLED);
    cv::line(
      cvBridge_->image, cv::Point(rightKneeX, rightKneeY), cv::Point(
        rightAnkleX,
        rightAnkleY), skeletonColor, 5,
      cv::FILLED);

    // Left leg
    cv::line(
      cvBridge_->image, cv::Point(leftHipX, leftHipY), cv::Point(
        leftKneeX,
        leftKneeY), skeletonColor, 5,
      cv::FILLED);
    cv::line(
      cvBridge_->image, cv::Point(leftKneeX, leftKneeY), cv::Point(
        leftAnkleX,
        leftAnkleY), skeletonColor, 5,
      cv::FILLED);

  }
}

void HumansDisplay::processMessage(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  hri_executor_->spin_some();

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
    texture_->addMessage(msg);
    return;
  }

  cvBridge_ = cv_bridge::toCvCopy(msg);

  if (show_faces_ || show_facial_landmarks_) {
    auto faces = hri_listener_->getFaces();
    for (auto const & face : faces) {
      if (face.second->valid()) {  // ensure the face fields are valid
        auto face_ptr = face.second;
        if (show_faces_) {
          auto roi = face_ptr->roi();
          cv::Point roi_tl(static_cast<int>(roi->x * msg->width),
            static_cast<int>(roi->y * msg->height));
          cv::Point roi_br(static_cast<int>((roi->x + roi->width) * msg->width),
            static_cast<int>((roi->y + roi->height) * msg->height));
          cv::rectangle(cvBridge_->image, roi_tl, roi_br, get_color_from_id(face.first), 5);
        }
        if (show_facial_landmarks_) {
          auto landmarks = *(face_ptr->facialLandmarks()); // boost::optional
          for (auto landmark : landmarks) {
            if (landmark.second.x > 0 || landmark.second.y > 0) {
              cv::circle(
                cvBridge_->image,
                cv::Point(
                  static_cast<int>(landmark.second.x * msg->width),
                  static_cast<int>(landmark.second.y * msg->height)),
                5,
                get_color_from_id(face.first), cv::FILLED);
            }
          }
        }
      }
    }
  }

  if (show_bodies_ || show_skeletons_) {
    auto bodies = hri_listener_->getBodies();
    for (auto const & body : bodies) {
      if (body.second->valid()) {  // ensure the body fields are valid
        auto body_ptr = body.second;
        if (show_bodies_) {
          auto roi = body_ptr->roi();
          cv::Point roi_tl(static_cast<int>(roi->x * msg->width),
            static_cast<int>(roi->y * msg->height));
          cv::Point roi_br(static_cast<int>((roi->x + roi->width) * msg->width),
            static_cast<int>((roi->y + roi->height) * msg->height));
          cv::rectangle(cvBridge_->image, roi_tl, roi_br, get_color_from_id(body.first), 5);
        }
        if (show_skeletons_) {
          auto skeleton = body_ptr->skeleton();
          if (skeleton) {
            drawSkeleton(body.first, msg->width, msg->height, *skeleton);
          }
        }
      }
    }
  }

  texture_->addMessage(cvBridge_->toImageMsg());
}

void HumansDisplay::setupScreenRectangle()
{
  static int count = 0;
  rviz_common::UniformStringStream ss;
  ss << "HumansDisplayObject" << count++;

  screen_rect_ = std::make_unique<Ogre::Rectangle2D>(true);
  screen_rect_->setRenderQueueGroup(Ogre::RENDER_QUEUE_OVERLAY - 1);
  screen_rect_->setCorners(-1.0f, 1.0f, 1.0f, -1.0f);

  ss << "Material";
  material_ = rviz_rendering::MaterialManager::createMaterialWithNoLighting(ss.str());
  material_->setSceneBlending(Ogre::SBT_REPLACE);
  material_->setDepthWriteEnabled(false);
  material_->setDepthCheckEnabled(false);

  Ogre::TextureUnitState * tu =
    material_->getTechnique(0)->getPass(0)->createTextureUnitState();
  tu->setTextureName(texture_->getName());
  tu->setTextureFiltering(Ogre::TFO_NONE);
  tu->setTextureAddressingMode(Ogre::TextureUnitState::TAM_CLAMP);

  material_->setCullingMode(Ogre::CULL_NONE);
  Ogre::AxisAlignedBox aabInf;
  aabInf.setInfinite();
  screen_rect_->setBoundingBox(aabInf);
  screen_rect_->setMaterial(material_);
}

void HumansDisplay::setupRenderPanel()
{
  render_panel_ = std::make_unique<rviz_common::RenderPanel>();
  render_panel_->resize(640, 480);
  render_panel_->initialize(context_);
  setAssociatedWidget(render_panel_.get());

  static int count = 0;
  render_panel_->getRenderWindow()->setObjectName(
    "HumansDisplayRenderWindow" + QString::number(count++));
}

}  // namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_hri_plugins::HumansDisplay, rviz_common::Display)
