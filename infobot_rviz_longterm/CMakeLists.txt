cmake_minimum_required(VERSION 2.8.3)
project(infobot_rviz_longterm)
find_package(catkin REQUIRED COMPONENTS rviz infobot_map_msgs octomap_msgs roscpp)
find_package(octomap REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread )
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rviz
)
include_directories(${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${OCTOMAP_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS} ${OCTOMAP_LIBRARY_DIRS})

## This plugin includes Qt widgets, so we must include Qt like so:
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
include(${QT_USE_FILE})

## I prefer the Qt signals and slots to avoid defining "emit", "slots",
## etc because they can conflict with boost signals, so define QT_NO_KEYWORDS here.
add_definitions(-DQT_NO_KEYWORDS)

## Here we specify which header files need to be run through "moc",
## Qt's meta-object compiler.
qt4_wrap_cpp(MOC_FILES
  src/point_cloud2_display.h
  src/point_cloud_common.h
  src/point_cloud_transformer.h
  src/point_cloud_transformers.h
  OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED
)

## Here we specify the list of source files, including the output of
## the previous command which is stored in ``${MOC_FILES}``.
set(SOURCE_FILES
  src/point_cloud2_display.cpp
  src/point_cloud_common.cpp
  src/point_cloud_transformers.cpp
  ${MOC_FILES}
)

## An rviz plugin is just a shared library, so here we declare the
## library to be called ``${PROJECT_NAME}`` (which is
## "rviz_plugin_tutorials", or whatever your version of this project
## is called) and specify the list of source files we collected above
## in ``${SOURCE_FILES}``.
add_library(${PROJECT_NAME} ${SOURCE_FILES})

## Link the library with whatever Qt libraries have been defined by
## the ``find_package(Qt4 ...)`` line above, and with whatever libraries
## catkin has included.
##
## Although this puts "rviz_plugin_tutorials" (or whatever you have
## called the project) as the name of the library, cmake knows it is a
## library and names the actual file something like
## "librviz_plugin_tutorials.so", or whatever is appropriate for your
## particular OS.
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${Boost_LIBRARIES} ${OCTOMAP_LIBRARIES} ${catkin_LIBRARIES} -ldefault_plugin)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
## END_TUTORIAL


## Install rules

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY icons/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)
