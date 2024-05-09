// Copyright 2012, Willow Garage, Inc.
// Copyright 2024, PAL Robotics, S.L.
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
#include <map>
#include <string>

#include <QObject>

#include <OgreMaterial.h>
#include <OgreRenderTargetListener.h>
#include <OgreSharedPtr.h>

#include <cv_bridge/cv_bridge.h>

#include <hri_msgs/msg/ids_list.hpp>
#include <hri_msgs/msg/normalized_point_of_interest2_d.hpp>
#include <hri/hri.hpp>

#include <rviz_common/message_filter_display.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/int_property.hpp>

#include <rviz_default_plugins/displays/image/ros_image_texture_iface.hpp>
#include <rviz_default_plugins/visibility_control.hpp>
#include <rviz_default_plugins/displays/image/image_transport_display.hpp>

#include <rclcpp/rclcpp.hpp>
#endif

namespace Ogre
{
class SceneNode;
class Rectangle2D;
}  // namespace Ogre

namespace rviz_hri_plugins
{
class HumansDisplay : public
  rviz_default_plugins::displays::ImageTransportDisplay<sensor_msgs::msg::Image>
{
  Q_OBJECT

public:
  explicit HumansDisplay(
    std::unique_ptr<rviz_default_plugins::displays::ROSImageTextureIface> texture);
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
  void drawSkeleton(
    std::string id, int width, int height, std::map<hri::SkeletalKeypoint,
    hri::PointOfInterest> & skeleton);

  /* This is called by incomingMessage(). */
  void processMessage(const sensor_msgs::msg::Image::ConstSharedPtr msg);

private:
  void setupScreenRectangle();
  void setupRenderPanel();

  void clear();

  std::unique_ptr<Ogre::Rectangle2D> screen_rect_;
  Ogre::MaterialPtr material_;

  std::unique_ptr<rviz_default_plugins::displays::ROSImageTextureIface> texture_;

  std::unique_ptr<rviz_common::RenderPanel> render_panel_;

  rviz_common::properties::BoolProperty * normalize_property_;
  rviz_common::properties::BoolProperty * show_faces_property_;
  rviz_common::properties::BoolProperty * show_facial_landmarks_property_;
  rviz_common::properties::BoolProperty * show_bodies_property_;
  rviz_common::properties::BoolProperty * show_skeletons_property_;
  rviz_common::properties::FloatProperty * min_property_;
  rviz_common::properties::FloatProperty * max_property_;
  rviz_common::properties::IntProperty * median_buffer_size_property_;

  bool got_float_image_;
  bool show_faces_;
  bool show_facial_landmarks_;
  bool show_bodies_;
  bool show_skeletons_;

  cv_bridge::CvImagePtr cvBridge_;

  rclcpp::Executor::SharedPtr hri_executor_;
  rclcpp::Node::SharedPtr hri_node_;
  std::shared_ptr<hri::HRIListener> hri_listener_;
};

}  // namespace rviz

#endif
