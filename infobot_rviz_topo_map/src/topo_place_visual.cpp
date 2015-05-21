#include "infobot_rviz_topo_map/topo_place_visual.h"

#include <boost/make_shared.hpp>
#include <boost/format.hpp>
#include <rviz/ogre_helpers/arrow.h>
#include <rviz/ogre_helpers/shape.h>
#include <rviz/ogre_helpers/movable_text.h>

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreQuaternion.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreEntity.h>

namespace infobot_rviz_topo_map
{

using boost::shared_ptr;
using boost::make_shared;
using infobot_topo_msgs::TopologicalPlace;
using infobot_topo_msgs::TopologicalView;


TopoPlaceVisual::TopoPlaceVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* map_frame_node)
    : scene_manager_(scene_manager), place_node_(0), place_id_text_(0)
{
  // Create a node for the metric map frame
  place_node_ = map_frame_node->createChildSceneNode();

  // Adding the shape
  shape_ = make_shared<rviz::Shape>(rviz::Shape::Cone, scene_manager_, place_node_);
  shape_->setScale(Ogre::Vector3(0.4, 0.4, 0.4));
  shape_->setOrientation(Ogre::Quaternion(1.0, 1.0, 1.0, 1.0));
  shape_->setPosition(Ogre::Vector3(0.0, 0.0, 0.2));
}


// -----------------------------------------------------
TopoPlaceVisual::~TopoPlaceVisual()
{
  if (place_node_)
    scene_manager_->destroySceneNode(place_node_);

  if (place_id_text_)
    delete place_id_text_;

  for (int i = 0; i < view_id_texts_.size(); ++i)
    delete view_id_texts_[i];

  for (int i = 0; i < view_id_text_nodes_.size(); ++i)
    scene_manager_->destroySceneNode(view_id_text_nodes_[i]);
}


// -----------------------------------------------------
void TopoPlaceVisual::setPlace(const TopologicalPlace& place,
                               bool show_place_id, bool show_view_id)
{
  const TopologicalView &first_view = place.views[0];

  // Move the place frame to the place position
  Ogre::Vector3 ogre_pos(first_view.pose.x, first_view.pose.y, 0.0);
  place_node_->setPosition(ogre_pos);

  // Remove old id text
  if (place_id_text_)
  {
    delete place_id_text_;
    place_id_text_ = 0;
  }

  // Add new place id text
  if (show_place_id)
  {
    place_id_text_ = new rviz::MovableText(boost::str(boost::format("%d") % place.place_id));
    place_id_text_->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_CENTER);
    place_id_text_->setCharacterHeight(0.2);
    place_id_text_->setLocalTranslation(Ogre::Vector3(0, 0, 0.5));
    place_node_->attachObject(place_id_text_);
  }

  // Remove old arrows
  view_arrows_.clear();
  for (int i = 0; i < view_id_texts_.size(); ++i)
    delete view_id_texts_[i];
  view_id_texts_.clear();
  for (int i = 0; i < view_id_text_nodes_.size(); ++i)
    scene_manager_->destroySceneNode(view_id_text_nodes_[i]);
  view_id_text_nodes_.clear();

  // Create new arrows
  for (int i = 0; i < place.views.size(); ++i)
  {
    const TopologicalView &view = place.views[i];

    Ogre::Quaternion view_orient(Ogre::Radian(view.pose.theta),
                                  Ogre::Vector3::UNIT_X);

    // Adding the arrow
    shared_ptr<rviz::Arrow> arrow = make_shared<rviz::Arrow>(scene_manager_, place_node_);
    arrow->setScale(Ogre::Vector3(0.3, 0.3, 0.3));
    arrow->setOrientation(Ogre::Quaternion(Ogre::Degree(90),
                                           Ogre::Vector3::UNIT_Y)*
                          Ogre::Quaternion(Ogre::Degree(180),
                                           Ogre::Vector3::UNIT_X)*
                          view_orient);
    view_arrows_.push_back(arrow);

    if (show_view_id)
    {
      // Adding arrow ID node
      Ogre::SceneNode* view_id_text_node = place_node_->createChildSceneNode();
      view_id_text_node->setPosition(Ogre::Vector3(0.4 * cos(view.pose.theta),
                                                   0.4 * sin(view.pose.theta),
                                                   0.05));
      view_id_text_nodes_.push_back(view_id_text_node);

      // Addding arrow ID
      rviz::MovableText *view_id_text = new rviz::MovableText(boost::str(boost::format("%d") % view.view_id));
      view_id_text->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_CENTER);
      view_id_text->setCharacterHeight(0.1);
      view_id_text_node->attachObject(view_id_text);
      view_id_texts_.push_back(view_id_text);
    }
  }
}

// -----------------------------------------------------
void TopoPlaceVisual::setColor(float r, float g, float b, float a)
{
  shape_->setColor(r, g, b, a);
  if (place_id_text_)
    place_id_text_->setColor(Ogre::ColourValue(0, 0, 0, a));
  for (int i = 0; i < view_arrows_.size(); ++i)
    view_arrows_[i]->setColor(Ogre::ColourValue(r, g, b, a));
  for (int i = 0; i < view_id_texts_.size(); ++i)
    view_id_texts_[i]->setColor(Ogre::ColourValue(0, 0, 0, a));
}


}  // namespace infobot_rviz_topo_map
