cmake_minimum_required(VERSION 2.8.3)
project(infobot_rviz_pose_tool)

find_package(catkin REQUIRED COMPONENTS rviz)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS rviz
  INCLUDE_DIRS
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  include/infobot_rviz_pose_tool/pose_tool.h
)

## Here we specify the list of source files
set(SOURCE_FILES src/pose_tool.cpp
                 ${MOC_FILES})

## Build libraries
add_library(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

install(FILES plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)
