cmake_minimum_required(VERSION 2.8.3)
project(shape_finder)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
  geometry_msgs
  topic_tools
  visualization_msgs
)

## System dependencies are found with CMake's conventions
find_package(PCL 1.8 REQUIRED)
set(PCL_INCLUDE_DIRS /usr/local/include/pcl-1.8)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )


generate_dynamic_reconfigure_options(
#  cfg/voxel_filter_node.cfg
#  cfg/passthrough_filter_node.cfg
#  cfg/plane_segmentation_node.cfg
  cfg/shape_finder_node.cfg
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_pcl_tutorial
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(my_pcl_tutorial
#   src/${PROJECT_NAME}/my_pcl_tutorial.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(my_pcl_tutorial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(my_pcl_tutorial_node src/my_pcl_tutorial_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(my_pcl_tutorial_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(my_pcl_tutorial_node
#   ${catkin_LIBRARIES}
# )

#add_executable(voxel_filter_node src/voxel_filter_node.cpp)
#add_executable(passthrough_filter_node src/passthrough_filter_node.cpp)
#add_executable(plane_segmentation_node src/plane_segmentation_node.cpp)
#add_executable(trianglemesh_node src/trianglemesh_node.cpp)
add_executable (shape_finder_node src/shape_finder_node.cpp)

#target_link_libraries(voxel_filter_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#target_link_libraries(passthrough_filter_node ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#target_link_libraries(plane_segmentation_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
#target_link_libraries(trianglemesh_node ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries (shape_finder_node ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

#add_dependencies(voxel_filter_node ${PROJECT_NAME}_gencfg)
#add_dependencies(passthrough_filter_node ${PROJECT_NAME}_gencfg)
#add_dependencies(plane_segmentation_node ${PROJECT_NAME}_gencfg)
#add_dependencies(trianglemesh_node ${PROJECT_NAME}_gencfg)
add_dependencies(shape_finder_node ${PROJECT_NAME}_gencfg)


