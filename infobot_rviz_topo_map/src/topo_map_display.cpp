#include "infobot_rviz_topo_map/topo_map_display.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>
#include <boost/make_shared.hpp>
#include <string>

#include "infobot_topo_msgs/TopologicalPlace.h"
#include "infobot_topo_msgs/TopologicalView.h"


namespace infobot_rviz_topo_map
{
using boost::mutex;
using infobot_topo_msgs::TopologicalMap;
using infobot_topo_msgs::TopologicalPlace;
using infobot_topo_msgs::TopologicalView;


// -----------------------------------------------------
TopoMapDisplay::TopoMapDisplay()
    : map_frame_node_(0), messages_received_(0)
{
  // Create properties
  color_property_ = new rviz::ColorProperty("Color", QColor( 255, 50, 50 ),
                                            "Color used to draw the places.",
                                            this, SLOT(updateColorAndAlpha()));

  alpha_property_ = new rviz::FloatProperty("Alpha", 1.0,
                                            "0 is fully transparent, 1.0 is fully opaque.",
                                            this, SLOT(updateColorAndAlpha()));
  metric_map_id_property_ = new rviz::StringProperty( "Metric map ID", "",
                                                      "ID of the currently displayed metric map.",
                                                      this, SLOT(updateMetricMapId()));
  show_place_ids_property_ = new rviz::BoolProperty("Show place IDs", true,
                                                    "",
                                                    this, SLOT(updateShowPlaceViewIds()));
  show_view_ids_property_ = new rviz::BoolProperty("Show view IDs", false,
                                                   "",
                                                   this, SLOT(updateShowPlaceViewIds()));
}


// -----------------------------------------------------
TopoMapDisplay::~TopoMapDisplay()
{
  // Delete the frame node
  if (map_frame_node_)
    context_->getSceneManager()->destroySceneNode(map_frame_node_);
}


// -----------------------------------------------------
void TopoMapDisplay::onInitialize()
{
  MFDClass::onInitialize();

  // Create a node for the metric map frame
  map_frame_node_ = scene_node_->createChildSceneNode();
}


// -----------------------------------------------------
void TopoMapDisplay::updateMetricMapInfo()
{
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame of the map.  If it fails, we can't
  // do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform("/map",
                                                 ros::Time(),  // Use latest transform
                                                 position, orientation))
  {
    ROS_ERROR("Error transforming from frame 'map' to frame '%s'",
              qPrintable(fixed_frame_));
    return;
  }

  // Create a frame node for the map
  map_frame_node_->setPosition(position);
  map_frame_node_->setOrientation(orientation);
}


// -----------------------------------------------------
void TopoMapDisplay::processTopoMap(const infobot_topo_msgs::TopologicalMap::ConstPtr& msg)
{
  mutex::scoped_lock lock(mutex_);

  // Clear the existing visuals
  visuals_.clear();

  // Remove all previous objects from the node
  map_frame_node_->detachAllObjects();

  // Do nothing further if no valid map
  if (!msg)
    return;

  // Current metric map id
  std::string cur_metric_map_id = metric_map_id_property_->getStdString();


  // Traverse the metric map and generate visuals for each place
  for (int i = 0; i < msg->places.size(); ++i)
  {
    const TopologicalPlace &place = msg->places[i];

    // If place is in the current metric map, show it
    if (place.metric_map_id == cur_metric_map_id)
    {
      boost::shared_ptr<TopoPlaceVisual> visual =
          boost::make_shared<TopoPlaceVisual>(context_->getSceneManager(), map_frame_node_);
      visual->setPlace(place,
                       show_place_ids_property_->getBool(),
                       show_view_ids_property_->getBool());

      // Set color
      float alpha = alpha_property_->getFloat();
      Ogre::ColourValue color = color_property_->getOgreColor();
      visual->setColor(color.r, color.g, color.b, alpha);

      // Add to vector of visuals
      visuals_.push_back(visual);
    }
  }
}


// -----------------------------------------------------
void TopoMapDisplay::updateColorAndAlpha()
{
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();

  for (size_t i = 0; i < visuals_.size(); i++)
  {
    visuals_[ i ]->setColor(color.r, color.g, color.b, alpha);
  }
}


// -----------------------------------------------------
void TopoMapDisplay::updateMetricMapId()
{
  updateMetricMapInfo();
  processTopoMap(topo_map_);
}


// -----------------------------------------------------
void TopoMapDisplay::updateShowPlaceViewIds()
{
  processTopoMap(topo_map_);
}


// -----------------------------------------------------
void TopoMapDisplay::reset()
{
  MFDClass::reset();
  visuals_.clear();
}


// -----------------------------------------------------
void TopoMapDisplay::processMessage(const infobot_topo_msgs::TopologicalMap::ConstPtr& msg)
{
  // Message count
  ++messages_received_;
  std::stringstream ss;
  ss << messages_received_ << " messages received";
  setStatus(rviz::StatusProperty::Ok, "Topic", ss.str().c_str());

  // Save the map for later
  topo_map_ = msg;

  // Set metric map ID to the same as topo map ID if not yet set
  if (metric_map_id_property_->getStdString().empty())
  {
    metric_map_id_property_->setValue(msg->topo_map_id.c_str());
  }

  updateMetricMapInfo();
  processTopoMap(msg);
}

}  // namespace infobot_rviz_topo_map


// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(infobot_rviz_topo_map::TopoMapDisplay, rviz::Display)
