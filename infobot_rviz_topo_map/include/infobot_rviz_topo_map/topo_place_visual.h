#ifndef INFOBOT_RVIZ_TOPO_MAP_TOPO_PLACE_VISUAL_H
#define INFOBOT_RVIZ_TOPO_MAP_TOPO_PLACE_VISUAL_H

#include <vector>

#include "infobot_topo_msgs/TopologicalPlace.h"


namespace Ogre
{
class Vector3;
class Quaternion;
class SceneManager;
class SceneNode;
class Entity;
}

namespace rviz
{
class Shape;
class MovableText;
class Arrow;
}

namespace infobot_rviz_topo_map
{

class TopoPlaceVisual
{
 public:
  // Constructor. Creates unconfigured visual stuff
  TopoPlaceVisual(Ogre::SceneManager* scene_manager,
                  Ogre::SceneNode* map_frame_node);

  // Destructor.  Removes the visual stuff from the scene.
  virtual ~TopoPlaceVisual();

  // Configure the visual to show the given place.
  void setPlace(const infobot_topo_msgs::TopologicalPlace& place,
                bool show_place_id, bool show_view_id);

  // Set the color and alpha of the visual, which are user-editable
  // parameters and therefore don't come from the Imu message.
  void setColor(float r, float g, float b, float a);


 private:
  // The SceneManager
  Ogre::SceneManager* scene_manager_;

  // A place coordinate system node
  Ogre::SceneNode* place_node_;

  // The object implementing the place shape
  boost::shared_ptr<rviz::Shape> shape_;

  /** Place ID text */
  rviz::MovableText *place_id_text_;

  /** View ID text */
  std::vector<boost::shared_ptr<rviz::Arrow> > view_arrows_;

  /** View ID text */
  std::vector<rviz::MovableText *> view_id_texts_;

  // View ID text nodes
  std::vector<Ogre::SceneNode*> view_id_text_nodes_;
};

}  // namespace infobot_rviz_topo_map


#endif
