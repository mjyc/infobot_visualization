#include "infobot_rviz_pose_tool/pose_tool.h"

#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"
#include "rviz/properties/bool_property.h"
#include "rviz/geometry.h"
#include "rviz/ogre_helpers/arrow.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/load_resource.h"
#include "rviz/render_panel.h"

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

#include <QApplication>
#include <QMessageBox>
#include <QClipboard>

#include <OGRE/OgrePlane.h>
#include <OGRE/OgreRay.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreViewport.h>

#include <string>


namespace infobot_rviz_pose_tool
{

PoseTool::PoseTool(): Tool(), arrow_(NULL)
{
  shortcut_key_ = 'r';

  topic_property_ = new rviz::StringProperty("Topic", "/clicked_pose",
                                             "The topic on which to publish the selected pose.",
                                             getPropertyContainer(), SLOT(updateTopic()), this);

  show_dialog_property_ = new rviz::BoolProperty("Show dialog box", true,
                                                 "Show dialog box with the selected pose.",
                                                 getPropertyContainer(), SLOT(updateShowDialog()), this);

  copy_property_ = new rviz::BoolProperty("Copy to clipboard", true,
                                          "Copy the selected pose to clipboard",
                                          getPropertyContainer(), SLOT(updateCopy()), this);
}


PoseTool::~PoseTool()
{
  delete arrow_;
}


void PoseTool::onInitialize()
{
  arrow_ = new rviz::Arrow(scene_manager_, NULL, 2.0f, 0.2f, 0.5f, 0.35f);
  arrow_->setColor(0.0f, 1.0f, 0.0f, 1.0f);
  arrow_->getSceneNode()->setVisible(false);

  setName("Get Pose");
  updateTopic();
}


void PoseTool::activate()
{
  setStatus("Click and drag mouse to set position/orientation.");
  state_ = Position;
}


void PoseTool::deactivate()
{
  arrow_->getSceneNode()->setVisible(false);
}


int PoseTool::processMouseEvent(rviz::ViewportMouseEvent& event)
{
  int flags = 0;

  if (event.leftDown())
  {
    ROS_ASSERT(state_ == Position);

    Ogre::Vector3 intersection;
    Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
    if (rviz::getPointOnPlaneFromWindowXY(event.viewport,
                                           ground_plane,
                                           event.x, event.y, intersection))
    {
      pos_ = intersection;
      arrow_->setPosition(pos_);

      state_ = Orientation;
      flags |= Render;
    }
  }
  else if (event.type == QEvent::MouseMove && event.left())
  {
    if (state_ == Orientation)
    {
      // Compute angle in x-y plane
      Ogre::Vector3 cur_pos;
      Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
      if (rviz::getPointOnPlaneFromWindowXY(event.viewport,
                                       ground_plane,
                                       event.x, event.y, cur_pos))
      {
        double angle = atan2(cur_pos.y - pos_.y, cur_pos.x - pos_.x);

        arrow_->getSceneNode()->setVisible(true);

        // We need base_orient, since the arrow goes along
        // the -z axis by default (for historical reasons)
        Ogre::Quaternion orient_x = Ogre::Quaternion(
          Ogre::Radian(-Ogre::Math::HALF_PI), Ogre::Vector3::UNIT_Y);

        arrow_->setOrientation(
          Ogre::Quaternion(Ogre::Radian(angle), Ogre::Vector3::UNIT_Z) * orient_x);

        flags |= Render;
      }
    }
  }
  else if (event.leftUp())
  {
    if (state_ == Orientation)
    {
      // Compute angle in x-y plane
      Ogre::Vector3 cur_pos;
      Ogre::Plane ground_plane(Ogre::Vector3::UNIT_Z, 0.0f);
      if (rviz::getPointOnPlaneFromWindowXY(event.viewport,
                                       ground_plane,
                                       event.x, event.y, cur_pos))
      {
        double angle = atan2(cur_pos.y - pos_.y, cur_pos.x - pos_.x);

        onPoseSet(pos_.x, pos_.y, angle);

        flags |= (Finished|Render);
      }
    }
  }

  return flags;
}


void PoseTool::updateTopic()
{
  pub_ = nh_.advertise<geometry_msgs::PoseStamped>(topic_property_->getStdString(), 1);
}

void PoseTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), fixed_frame);
  geometry_msgs::PoseStamped pose;
  tf::poseStampedTFToMsg(p, pose);
  ROS_INFO("Publishing pose: Frame:%s, Position(%.3f, %.3f, %.3f), "
           "Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
           pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
           pose.pose.orientation.x, pose.pose.orientation.y,
           pose.pose.orientation.z, pose.pose.orientation.w, theta);
  pub_.publish(pose);

  QString poseStr = QString::number(pos_.x) + " " +
    QString::number(pos_.y) + " " +
    QString::number(theta);

  if (show_dialog_property_->getBool())
  {
    QMessageBox* msgBox = new QMessageBox(NULL);
    // Makes sure the msgbox is deleted automatically when closed
    msgBox->setAttribute(Qt::WA_DeleteOnClose);
    msgBox->setStandardButtons(QMessageBox::Ok);
    msgBox->setWindowTitle("Selected Pose");
    msgBox->setText(poseStr);
    msgBox->setModal(false);
    msgBox->open(NULL, NULL);
  }

  if (copy_property_->getBool())
  {
    QClipboard *clipboard = QApplication::clipboard();
    clipboard->setText(poseStr);
  }
}

}  // end namespace infobot_rviz_pose_tool

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(infobot_rviz_pose_tool::PoseTool, rviz::Tool)
