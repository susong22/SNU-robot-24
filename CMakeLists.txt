cmake_minimum_required(VERSION 3.0.2)
project(lidar_package)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  tf
)

## Declare a catkin package
catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Add executables for main node (main.cpp)
add_executable(main_node src/lidar.cpp src/main.cpp src/navigation.cpp)

## Link libraries to the main_node executable
target_link_libraries(main_node
  ${catkin_LIBRARIES}
  ${tf_LIBRARIES}
)

## Mark executables for installation
install(TARGETS main_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
