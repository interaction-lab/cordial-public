cmake_minimum_required(VERSION 2.4.6)
project(cordial_manager)

catkin_python_setup()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  audio_common_msgs
  message_generation
)

add_message_files(
  FILES
  PlayRequest.msg
)


add_service_files(
  FILES
  PlayAudio.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  audio_common_msgs
)


catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)






