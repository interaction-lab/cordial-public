cmake_minimum_required(VERSION 2.4.6)
project(cordial_lex)

catkin_python_setup()

catkin_package()

find_package(catkin REQUIRED COMPONENTS
  aws_common
  aws_ros1_common
  lex_common
  lex_common_msgs
  roscpp
  std_msgs
  lex_node
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cordial_lex
  CATKIN_DEPENDS aws_ros1_common lex_common_msgs roscpp std_msgs
  #DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${AWSSDK_INCLUDE_DIR}
  ${lex_common_INCLUDE_DIR}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY scripts src
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


