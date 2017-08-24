cmake_minimum_required(VERSION 2.8.3)
project(yumi_cameras)

add_compile_options(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(
  catkin REQUIRED COMPONENTS
    abb_egm_interface
    abb_rws_interface
    message_generation
    simple_message
)

# Declare catkin package
catkin_package(
  CATKIN_DEPENDS
    abb_egm_interface
    abb_rws_interface
    simple_message
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include ${catkin_INCLUDE_DIRS})



add_executable(rws_continuous_trigger src/rws_continuous_trigger.cpp)
target_link_libraries(rws_continuous_trigger ${catkin_LIBRARIES})


