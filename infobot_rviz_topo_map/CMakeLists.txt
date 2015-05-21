cmake_minimum_required(VERSION 2.8.3)
project(infobot_rviz_topo_map)

find_package(catkin REQUIRED COMPONENTS roscpp rviz infobot_topo_msgs)
find_package(Boost REQUIRED)

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 4.6 REQUIRED COMPONENTS QtCore QtGui)
include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  include/infobot_rviz_topo_map/topo_map_display.h
)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp rviz infobot_topo_msgs
  INCLUDE_DIRS include
  LIBRARIES
)

include_directories(SYSTEM ${BOOST_INCLUDE_DIRS}
                    ${catkin_INCLUDE_DIRS})
include_directories(include)

## Here we specify the list of source files
set(SOURCE_FILES src/topo_map_display.cpp src/topo_place_visual.cpp
                 ${MOC_FILES})

## Build libraries
add_library(${PROJECT_NAME} ${SOURCE_FILES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

# Install
install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
