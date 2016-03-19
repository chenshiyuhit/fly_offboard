cmake_minimum_required(VERSION 2.8.3)
project(fly_offboard)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cmake_modules
)

find_package(Eigen REQUIRED)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES fly_shape
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(include  ${catkin_INCLUDE_DIRS})


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/./bin)


## Declare a cpp library
# add_library(flyline
#   src/fly_line.cpp
# )

 add_executable(flyline src/fly_line.cpp)
 add_dependencies(flyline fly_offboard_generate_messages_cpp)
 target_link_libraries(flyline  ${catkin_LIBRARIES})


 add_executable(flyloiter src/fly_loiter.cpp)
 add_dependencies(flyloiter fly_offboard_generate_messages_cpp)
 target_link_libraries(flyloiter  ${catkin_LIBRARIES})


 add_executable(flysitl src/fly_sitl.cpp)
 add_dependencies(flysitl fly_offboard_generate_messages_cpp)
 target_link_libraries(flysitl  ${catkin_LIBRARIES})

 add_executable(flytrack src/fly_track.cpp)
 add_dependencies(flytrack fly_offboard_generate_messages_cpp)
 target_link_libraries(flytrack  ${catkin_LIBRARIES})
 
 
#set optimization level 
set(CMAKE_BUILD_TYPE Release)
