/*
 * Copyright (c) 2012, Willow Garage, Inc.
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

#ifndef INFOBOT_RVIZ_MAP_PROBABILITY_MAP_DISPLAY_H
#define INFOBOT_RVIZ_MAP_PROBABILITY_MAP_DISPLAY_H

#include <string>
#include <vector>

#ifndef Q_MOC_RUN
#include <boost/thread/thread.hpp>

#include <OgreTexture.h>
#include <OgreMaterial.h>
#include <OgreVector3.h>
#include <OgreSharedPtr.h>
#endif

#include <infobot_map_msgs/ProbabilityMapMetaData.h>
#include <ros/time.h>

#include <infobot_map_msgs/ProbabilityGrid.h>
// #include <infobot_map_msgs/ProbabilityGridUpdate.h>

#include <rviz/display.h>

namespace Ogre
{
class ManualObject;
}

namespace rviz
{
class EnumProperty;
class FloatProperty;
class IntProperty;
class Property;
class QuaternionProperty;
class RosTopicProperty;
class VectorProperty;
}

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace infobot_rviz_map
{

/**
 * \class ProbabilityMapDisplay
 * \brief Displays a map along the XY plane.
 */
class ProbabilityMapDisplay: public rviz::Display
{
  Q_OBJECT
public:
  ProbabilityMapDisplay();
  virtual ~ProbabilityMapDisplay();

  // Overrides from Display
  virtual void onInitialize();
  virtual void fixedFrameChanged();
  virtual void reset();

  float getResolution()
  {
    return resolution_;
  }
  int getWidth()
  {
    return width_;
  }
  int getHeight()
  {
    return height_;
  }

  virtual void setTopic(const QString &topic, const QString &datatype);

Q_SIGNALS:
  /** @brief Emitted when a new map is received*/
  void mapUpdated();

protected Q_SLOTS:
  void updateAlpha();
  void updateTopic();
  void updateDrawUnder();
  void updatePalette();
  /** @brief Show current_map_ in the scene. */
  void showMap();

protected:
  // overrides from Display
  virtual void onEnable();
  virtual void onDisable();

  virtual void subscribe();
  virtual void unsubscribe();

  /** @brief Copy msg into current_map_ and call showMap(). */
  void incomingMap(const infobot_map_msgs::ProbabilityGrid::ConstPtr& msg);

  /** @brief Copy update's data into current_map_ and call showMap(). */
  // void incomingUpdate(const infobot_map_msgs::ProbabilityGridUpdate::ConstPtr& update);

  void clear();

  void transformMap();

  Ogre::ManualObject* manual_object_;
  Ogre::TexturePtr texture_;
  std::vector<Ogre::TexturePtr> palette_textures_;
  std::vector<bool> color_scheme_transparency_;
  Ogre::MaterialPtr material_;
  bool loaded_;

  std::string topic_;
  float resolution_;
  int width_;
  int height_;
  std::string frame_;
  infobot_map_msgs::ProbabilityGrid current_map_;

  ros::Subscriber map_sub_;
  ros::Subscriber update_sub_;

  rviz::RosTopicProperty* topic_property_;
  rviz::FloatProperty* resolution_property_;
  rviz::IntProperty* width_property_;
  rviz::IntProperty* height_property_;
  rviz::VectorProperty* position_property_;
  rviz::QuaternionProperty* orientation_property_;
  rviz::FloatProperty* alpha_property_;
  rviz::Property* draw_under_property_;
  rviz::EnumProperty* color_scheme_property_;
};

}  // namespace infobot_rviz_map

#endif
