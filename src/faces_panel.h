/*
 * Copyright (c) 2011, Willow Garage, Inc.
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
#ifndef FACES_PANEL_H
#define FACES_PANEL_H

#ifndef Q_MOC_RUN
# include <vector>
# include <string>

# include <ros/ros.h>

# include <rviz/panel.h>

# include <hri_msgs/IdsList.h>
# include <hri_msgs/RegionOfInterestStamped.h>
# include <sensor_msgs/RegionOfInterest.h>
# include <sensor_msgs/Image.h>
# include <cv_bridge/cv_bridge.h>

# include <QPixmap> 
# include <QLabel>
#endif

class QLineEdit;

namespace hri_rviz
{

class ImageWidget;

// BEGIN_TUTORIAL
// Here we declare our new subclass of rviz::Panel.  Every panel which
// can be added via the Panels/Add_New_Panel menu is a subclass of
// rviz::Panel.
//
// FacesPanel will show two text-entry field to set the input topics
// and an image label, to display camera images + bounding box for the 
// detected face.
class FacesPanel: public rviz::Panel
{

Q_OBJECT
public:
  // QWidget subclass constructors usually take a parent widget
  // parameter (which usually defaults to 0).  At the same time,
  // pluginlib::ClassLoader creates instances by calling the default
  // constructor (with no arguments).  Taking the parameter and giving
  // a default of 0 lets the default constructor work and also lets
  // someone using the class for something else to pass in a parent
  // widget as they normally would with Qt.
  FacesPanel( QWidget* parent = 0 );

  // Now we declare overrides of rviz::Panel functions for saving and
  // loading data from the config file.  Here the data is the
  // topic name.
  virtual void load( const rviz::Config& config );
  virtual void save( rviz::Config config ) const;

protected Q_SLOTS:
  // update[Object]Topic() reads the topic name from the QLineEdit and calls
  // setTopic(...) method with the result.
  void updateIdsTopic();
  void updateImgTopic();

  // Then we finish up with protected member variables.
protected:
  // One-line text editors for entering the input ROS topic name.
  QLineEdit* ids_input_topic_editor_;
  QLineEdit* img_input_topic_editor_;

  // The current name of the input topics.
  QString ids_input_topic_;
  QString img_input_topic_;

  //Qt object for image displaying
  QLabel *imageLabel;
  //cv_bridge object, necessary to elaborate 
  //the incoming image, which is a sensor_msgs::Image
  cv_bridge::CvImagePtr cvBridge_;
  cv::Rect rect;

  // The ROS subscriber for the list of detected faces ids.
  ros::Subscriber idsList_subscriber_;

  // The ROS subscriber for the camera recorded image.
  ros::Subscriber img_subscriber_;

  // The ROS subscriber for the currently detected face roi.
  ros::Subscriber face_subscriber_;

  // The ROS node handle.
  ros::NodeHandle nh_;

  // Structures needed to update the id of the detected face
  // and the relative roi. 
  std::vector<std::string> ids_;
  std::string current_id_; //Just for one face detection
  hri_msgs::RegionOfInterestStamped roi_;

  //Declaration of the various ROS callbacks 
  void faceCallback(const hri_msgs::RegionOfInterestStampedConstPtr& msg);
  void imgCallback(const sensor_msgs::ImageConstPtr& msg);
  void idsCallback(const hri_msgs::IdsListConstPtr& msg);

  //Declaration of the setTopic(...) function
  template<class M>
  void setTopic( const QString& new_topic, QString& topic_to_set, ros::Subscriber& sub_to_set, void(FacesPanel::*fp)(const boost::shared_ptr<M const>&));
  // END_TUTORIAL
};

} // end namespace hri_rviz

#endif // FACES_PANEL_H
