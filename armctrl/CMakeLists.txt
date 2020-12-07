cmake_minimum_required(VERSION 2.8.3)
project(armctl)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  controller_interface
  pluginlib
  roscpp
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_package( INCLUDE_DIRS src 
	LIBRARIES my_controller  
	CATKIN_DEPENDS pluginlib roscpp std_msgs
  DEPENDS Boost
)
add_library(armctl_lib src/armctl.cpp )
target_link_libraries( armctl_lib ${catkin_LIBRARIES} )

add_executable(${PROJECT_NAME}_node src/armctl.cpp)
target_link_libraries(${PROJECT_NAME}_node ${Boost_DATE_TIME_LIBRARY} ${catkin_LIBRARIES})
add_dependencies(${PROJECT_NAME}_node  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
