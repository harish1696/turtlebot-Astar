cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_spawn)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS 
    roscpp 
    geometry_msgs 
    move_base_msgs
)

## Declare a catkin package
catkin_package()

# Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(walker src/main.cpp)
target_link_libraries(walker ${catkin_LIBRARIES})