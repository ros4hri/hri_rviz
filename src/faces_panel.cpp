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

#include <stdio.h>

#include <QPainter>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTimer>
#include <QImage>

#include <opencv2/core/mat.hpp>

#include <geometry_msgs/Twist.h>

#include "faces_panel.h"

namespace hri_rviz
{

// Here is the implementation of the FacesPanel class.  FacesPanel
// has these responsibilities:
//
// - Display incoming images from input_topic including graphic
//   aspects conveying the information provided by hri_fullbody
FacesPanel::FacesPanel( QWidget* parent )
  : rviz::Panel( parent )
{
  // Next we lay out the "Input topic" text entry field using a
  // QLabel and a QLineEdit in a QHBoxLayout.
  QHBoxLayout* ids_topic_layout = new QHBoxLayout;
  ids_topic_layout->addWidget( new QLabel( "Ids Topic:" ));
  ids_input_topic_editor_ = new QLineEdit;
  ids_topic_layout->addWidget( ids_input_topic_editor_ );

  QHBoxLayout* img_topic_layout = new QHBoxLayout;
  img_topic_layout->addWidget(new QLabel( "Image Topic:"));
  img_input_topic_editor_ = new QLineEdit;
  img_topic_layout->addWidget(img_input_topic_editor_);

  imageLabel = new QLabel();

  QVBoxLayout* layout = new QVBoxLayout;
  layout->addLayout(ids_topic_layout);
  layout->addLayout(img_topic_layout);
  layout->addWidget(imageLabel);
  setLayout(layout);

  // Next we make signal/slot connections.
  connect(ids_input_topic_editor_, 
          SIGNAL(editingFinished()), 
          this, 
          SLOT(updateIdsTopic()));
  connect(img_input_topic_editor_, 
          SIGNAL(editingFinished()), 
          this, 
          SLOT(updateImgTopic()));

  // We set the roi width to -1, which means that no roi is currently present
  roi_.roi.width = -1;
}


// Read the topic name from the QLineEdit and call setTopic(...) with the
// results.  This is connected to QLineEdit::editingFinished() which
// fires when the user presses Enter or Tab or otherwise moves focus
// away.
void FacesPanel::updateIdsTopic()
{
  setTopic(ids_input_topic_editor_->text(), 
           ids_input_topic_, 
           idsList_subscriber_, 
           &FacesPanel::idsCallback);
}

void FacesPanel::updateImgTopic()
{
  setTopic(img_input_topic_editor_->text(), 
           img_input_topic_, 
           img_subscriber_, 
           &FacesPanel::imgCallback);
}

// Publish the control velocities if ROS is not shutting down and the
// publisher is ready with a valid topic name.
void FacesPanel::idsCallback(const hri_msgs::IdsListConstPtr& msg)
{
  if( ros::ok() ){
    ids_ = msg->ids;
    if(!ids_.empty() && ids_[0]!="" && ids_[0]!=current_id_){
      std::string roi_topic = "/humans/faces/"+ids_[0]+"/roi";
      face_subscriber_.shutdown();
      face_subscriber_ = nh_.subscribe(roi_topic, 
                                       1, 
                                       &FacesPanel::faceCallback, 
                                       this);
    }
    else
      roi_.roi.width = -1;
  }
}

void FacesPanel::faceCallback(
  const hri_msgs::RegionOfInterestStampedConstPtr& msg)
{
  roi_.roi.x_offset = msg->roi.x_offset;
  roi_.roi.y_offset = msg->roi.y_offset;
  roi_.roi.width = msg->roi.width;
  roi_.roi.height = msg->roi.height;
  roi_.header = msg->header;
}

void FacesPanel::imgCallback(const sensor_msgs::ImageConstPtr& msg)
{
  int w = 300;
  int h = 300;

  cvBridge_ = cv_bridge::toCvCopy(msg);
  cv::Mat image = cvBridge_->image;

  if(roi_.roi.width > 0){
    rect = cv::Rect(int(roi_.roi.x_offset), 
                    int(roi_.roi.y_offset), 
                    int(roi_.roi.width), 
                    int(roi_.roi.height));
    cv::rectangle(cvBridge_->image, rect, cv::Scalar(0, 255, 0), 5);
  }

  imageLabel->setPixmap(
    QPixmap::fromImage(QImage(image.data, 
                              image.cols, 
                              image.rows, 
                              image.step, 
                              QImage::Format_RGB888)).scaled(w,h,Qt::KeepAspectRatio));
}

// Set the topic name we are publishing to.
template <class M>
void FacesPanel::setTopic( const QString& new_topic, 
                      QString& topic_to_set, 
                      ros::Subscriber& sub_to_set, 
                      void(FacesPanel::*fp)(const boost::shared_ptr<M const>&))
{
  // Only take action if the name has changed.
  if( new_topic != topic_to_set )
  {
    topic_to_set = new_topic;
    sub_to_set.shutdown();
    // If the topic is the empty string, don't publish anything.
    if( topic_to_set != "" )
    {
      sub_to_set.shutdown();
      sub_to_set = nh_.subscribe<M>(new_topic.toStdString(), 1, fp, this);
    }
    // rviz::Panel defines the configChanged() signal.  Emitting it
    // tells RViz that something in this panel has changed that will
    // affect a saved config file.  Ultimately this signal can cause
    // QWidget::setWindowModified(true) to be called on the top-level
    // rviz::VisualizationFrame, which causes a little asterisk ("*")
    // to show in the window's title bar indicating unsaved changes.
    Q_EMIT configChanged();
  }

}

// Save all configuration data from this panel to the given
// Config object.  It is important here that you call save()
// on the parent class so the class id and panel name get saved.
void FacesPanel::save( rviz::Config config ) const
{
  rviz::Panel::save( config );
  config.mapSetValue( "IdsTopic", ids_input_topic_ );
  config.mapSetValue( "ImgTopic", img_input_topic_ );
}

// Load all configuration data for this panel from the given Config object.
void FacesPanel::load( const rviz::Config& config )
{
  rviz::Panel::load( config );
  QString idsTopic, imgTopic;
  if( config.mapGetString( "IdsTopic", &idsTopic ))
  {
    ids_input_topic_editor_->setText( idsTopic );
    updateIdsTopic();
  }
  if( config.mapGetString( "ImgTopic", &imgTopic ))
  {
    ids_input_topic_editor_->setText( imgTopic );
    updateImgTopic();
  }
}

} // end namespace hri_rviz

// Tell pluginlib about this class.  Every class which should be
// loadable by pluginlib::ClassLoader must have these two lines
// compiled in its .cpp file, outside of any namespace scope.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(hri_rviz::FacesPanel,rviz::Panel )
// END_TUTORIAL
