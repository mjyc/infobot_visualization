#ifndef __INFOBOT_RVIZ_POSE_TOOL__POSE_TOOL__
#define __INFOBOT_RVIZ_POSE_TOOL__POSE_TOOL__

#include <OGRE/OgreVector3.h>

# include <QCursor>
# include <ros/ros.h>
# include "rviz/tool.h"

namespace rviz
{
  class Arrow;
  class DisplayContext;
  class StringProperty;
  class BoolProperty;
}


namespace infobot_rviz_pose_tool
{

class PoseTool: public rviz::Tool
{
Q_OBJECT

public:
  PoseTool();
  virtual ~PoseTool();

  virtual void onInitialize();
  virtual void activate();
  virtual void deactivate();

  virtual int processMouseEvent(rviz::ViewportMouseEvent& event);

private Q_SLOTS:
  void updateTopic();
  void updateShowDialog() {}
  void updateCopy() {}

private:
  void onPoseSet(double x, double y, double theta);

  rviz::Arrow* arrow_;

  enum State
  {
    Position,
    Orientation
  };
  State state_;

  Ogre::Vector3 pos_;

  ros::NodeHandle nh_;
  ros::Publisher pub_;

  rviz::StringProperty* topic_property_;
  rviz::BoolProperty* show_dialog_property_;
  rviz::BoolProperty* copy_property_;
};

}  // namespace infobot_rviz_pose_tool

#endif


