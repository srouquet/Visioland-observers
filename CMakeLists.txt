cmake_minimum_required(VERSION 2.8.3)
project(visioland_observers)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  ressac_msgs
)


## Generate messages in the 'msg' folder
#add_message_files(
# FILES
#)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES visioland_observers
  CATKIN_DEPENDS message_runtime
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  include
  #Eigen dependencies
  "/usr/include/eigen3"
)

add_library(visioland_observers src/observer_base.cpp)
add_executable(test_node src/test_node.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES} visioland_observers)
