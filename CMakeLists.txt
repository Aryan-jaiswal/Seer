cmake_minimum_required(VERSION 2.8.3)
project(seer)



find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  cv_bridge
  image_transport
  message_generation
  tf
)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependencies might have been
##     pulled in transitively but can be declared for certainty nonetheless:
##     * add a build_depend tag for "message_generation"
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
add_message_files(
   FILES
   AprilTag.msg
   AprilTagList.msg
)



## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  sensor_msgs
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
#  LIBRARIES april_tag
  CATKIN_DEPENDS roscpp sensor_msgs message_runtime
  DEPENDS eigen3
)

#include_directories(include /usr/include/eigen3)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  src/arucotags

)
file(GLOB SOURCE_FILES1 "src/apriltags/*.cc")

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD 11) # C++11...
set(CMAKE_CXX_STANDARD_REQUIRED ON) #...is required...
set(CMAKE_CXX_EXTENSIONS ON) #...with compiler extensions like gnu++11

include(cmake/options.cmake)
include(cmake/findDependencies.cmake)
include(cmake/compilerOptions.cmake)

add_subdirectory(src/arucotags)
catkin_package()


add_executable(april_tag_node src/april_tag_node.cpp src/rigidtransform.cc src/KalmanFilter.cc  
${SOURCE_FILES1})
target_link_libraries(april_tag_node 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES} 
  popt 
  config++ 
)
add_executable(aruco_tag_node src/aruco_tag_node.cpp src/rigidtransform.cc)
target_link_libraries(aruco_tag_node
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES} 
  popt 
  config++ 
  aruco 
  opencv_calib3d
)


#add_executable(publisher src/publisher.cpp)
#target_link_libraries(publisher ${catkin_LIBRARIES} aruco opencv_calib3d)

