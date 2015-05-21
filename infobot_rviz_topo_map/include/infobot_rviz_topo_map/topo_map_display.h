#ifndef INFOBOT_RVIZ_TOPO_MAP_TOPO_MAP_DISPLAY_H
#define INFOBOT_RVIZ_TOPO_MAP_TOPO_MAP_DISPLAY_H

#include "infobot_rviz_topo_map/topo_place_visual.h"
#include <infobot_topo_msgs/TopologicalMap.h>

#include <message_filters/subscriber.h>
#include <tf/message_filter.h>
#include <rviz/display.h>
#include <rviz/visualization_manager.h>
#include <rviz/properties/property.h>
#include <rviz/frame_manager.h>
#include <rviz/message_filter_display.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/string_property.h>

#include <string>
#include <set>
#include <vector>

namespace infobot_rviz_topo_map
{

class TopoMapDisplay:
    public rviz::MessageFilterDisplay<infobot_topo_msgs::TopologicalMap>
{
Q_OBJECT

public:
  TopoMapDisplay();
  virtual ~TopoMapDisplay();

  virtual void onInitialize();
  virtual void reset();

private Q_SLOTS:
  void updateColorAndAlpha();
  void updateMetricMapId();
  void updateShowPlaceViewIds();

private:
  void processTopoMap(const infobot_topo_msgs::TopologicalMap::ConstPtr& msg);

  void updateMetricMapInfo();

  virtual void processMessage(const infobot_topo_msgs::TopologicalMap::ConstPtr& msg);


private:
  int messages_received_;

  /** Current topololgical map. */
  infobot_topo_msgs::TopologicalMap::ConstPtr topo_map_;

  // Properties
  rviz::ColorProperty* color_property_;
  rviz::StringProperty* metric_map_id_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::BoolProperty* show_place_ids_property_;
  rviz::BoolProperty* show_view_ids_property_;

  // Visuals
  std::vector<boost::shared_ptr<TopoPlaceVisual> > visuals_;

  // A SceneNode whose pose is set to match the coordinate frame of
  // the current metric map
  Ogre::SceneNode* map_frame_node_;

  /** General mutex to prevent problems with concurrent updates. */
  boost::mutex mutex_;
};

}  // namespace infobot_rviz_topo_map

#endif
