cmake_minimum_required(VERSION 2.8.3)
project(demo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
        roscpp 
        rospy
        tf 
        cv_bridge 
        image_transport 
        visualization_msgs
        image_geometry 
        nodelet
        std_msgs
        geometry_msgs
        actionlib_msgs
        message_generation)

## System dependencies
find_package(OpenCV REQUIRED)

find_package(Eigen3 REQUIRED)

message(STATUS "OpenCV version: ${OpenCV_VERSION}")
if(2.9.0 VERSION_GREATER ${OpenCV_VERSION})
    message(FATAL "OpenCV3 required!")
endif()

## Generate actions in the 'action' folder
 add_action_files(
   FILES
   robot_geo.action
 )

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
   geometry_msgs
 )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ros_markers
  CATKIN_DEPENDS 
    tf
    nodelet
  DEPENDS OpenCV
  LIBRARIES 
)


###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})


add_definitions(-std=c++11)

set(LIBS 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

add_executable(pandaJacob src/jacob.cpp)
target_include_directories(pandaJacob PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} /home/prajna/catkin_ws/devel/include )
target_link_libraries(pandaJacob
   ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(DIRECTORY
    launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY
    rviz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
install(DIRECTORY
    urdfs/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdfs
)
install(DIRECTORY
    meshes/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)

catkin_install_python(PROGRAMS
    nodes/rviz_viewer.py
    nodes/start_here.py
    nodes/pc_publish
    nodes/static_publisher
    nodes/simulator
    #nodes/turtlesim_dkey
    #nodes/turtlesim_akey
    #nodes/trlistener
    #nodes/tr_listener_q
    #nodes/trlistener_combined
    #nodes/rot_trans
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(TARGETS pandaJacob
  RUNTIME DESTINATION lib/demo
)
