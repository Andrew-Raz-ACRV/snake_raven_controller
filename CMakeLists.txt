cmake_minimum_required(VERSION 2.8.3)
project(snake_raven_controller)
## originally project(AutoCircle_generater)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs message_generation)

## Add in message and service files
add_message_files(FILES raven_jointmove.msg ravenstate.msg snakeraven_state.msg)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

## Build talker and listeneri
include_directories(include ${catkin_INCLUDE_DIRS})

##originally talkerAutoCircle
add_executable(talkersnakeraven src/talker.cpp  src/Raven_Controller.cpp src/SnakeRaven.cpp)
## src/Raven_PathPlanner.cpp
target_link_libraries(talkersnakeraven ${catkin_LIBRARIES})
add_dependencies(talkersnakeraven snakeraven_test_generate_messages_cpp)
## was originally: AutoCircle_test_generate_messages_cpp

add_executable(listenersnakeraven src/listener.cpp)
target_link_libraries(listenersnakeraven ${catkin_LIBRARIES})
add_dependencies(listenersnakeraven snakeraven_test_generate_messages_cpp)
