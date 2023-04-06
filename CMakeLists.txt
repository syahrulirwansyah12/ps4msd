cmake_minimum_required(VERSION 3.0.2)
project(ps4msd)

find_package(catkin REQUIRED COMPONENTS
<<<<<<< HEAD
  geometry_msgs
=======
>>>>>>> 84b01099d2c327eb33cdd69295f60b3c978f6f48
  rospy
  std_msgs
  message_generation
)

<<<<<<< HEAD
=======
################################################
## Declare ROS messages, services and actions ##
################################################

>>>>>>> 84b01099d2c327eb33cdd69295f60b3c978f6f48

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Joystate.msg
)

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
generate_messages(
  DEPENDENCIES
<<<<<<< HEAD
  geometry_msgs std_msgs
=======
  std_msgs  # Or other packages containing msgs
>>>>>>> 84b01099d2c327eb33cdd69295f60b3c978f6f48
)

catkin_package(
#  INCLUDE_DIRS include
<<<<<<< HEAD
 LIBRARIES ps4msd
 CATKIN_DEPENDS geometry_msgs rospy std_msgs message_runtime
 DEPENDS system_lib
)

=======
#  LIBRARIES ps4msd
 CATKIN_DEPENDS rospy message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

>>>>>>> 84b01099d2c327eb33cdd69295f60b3c978f6f48
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
<<<<<<< HEAD
#   src/${PROJECT_NAME}/ps4msd.cpp
=======
#   src/${PROJECT_NAME}/PS4MSD.cpp
>>>>>>> 84b01099d2c327eb33cdd69295f60b3c978f6f48
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
<<<<<<< HEAD
# add_executable(${PROJECT_NAME}_node src/ps4msd_node.cpp)
=======
# add_executable(${PROJECT_NAME}_node src/PS4MSD_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
>>>>>>> 84b01099d2c327eb33cdd69295f60b3c978f6f48
