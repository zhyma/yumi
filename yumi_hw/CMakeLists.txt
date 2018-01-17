cmake_minimum_required(VERSION 2.8.3)
project(yumi_hw)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
add_definitions(-DROS=1)           # build using ROS libraries 
add_definitions(-DLINUXSOCKETS=1)  # build using LINUX SOCKETS libraries 

find_package(Boost REQUIRED COMPONENTS system thread QUIET)

find_package(abb_egm_interface QUIET) 
find_package(abb_rws_interface QUIET)
LIST(APPEND CATKIN_DEPENDS_LIST 
  cmake_modules
  control_toolbox
  controller_interface
  controller_manager
  hardware_interface
  joint_limits_interface
  kdl_parser
  realtime_tools
  roscpp
  std_msgs
  tf
  transmission_interface
  urdf
  simple_message
  message_generation
<<<<<<< HEAD
  abb_egm_interface
  abb_rws_interface
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS system thread)

# The definition is copied from the CMakeList for the simple_message package.
add_definitions(-DROS=1)           #build using ROS libraries
add_definitions(-DLINUXSOCKETS=1)  #build using LINUX SOCKETS libraries


## Generate services in the 'srv' folder
add_service_files(
  FILES
  YumiGrasp.srv
)
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS cmake_modules control_toolbox controller_interface controller_manager hardware_interface joint_limits_interface kdl_parser realtime_tools roscpp std_msgs tf transmission_interface urdf simple_message abb_egm_interface abb_rws_interface
#  DEPENDS gazebo
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/yumi_hw.cpp
  src/yumi_hw_rws.cpp
  src/yumi_hw_egm.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(yumi_hw ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

## Declare a C++ executable
add_executable(yumi_hw_ifce_node src/yumi_hw_ifce_node.cpp)
add_executable(yumi_gripper_node src/yumi_gripper_node.cpp)


## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(yumi_gripper_node ${PROJECT_NAME}_generate_messages_cpp)


## Specify libraries to link a library or executable target against
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} )
target_link_libraries( yumi_hw_ifce_node ${catkin_LIBRARIES} ${PROJECT_NAME} simple_message)
target_link_libraries( yumi_gripper_node ${catkin_LIBRARIES} ${PROJECT_NAME} simple_message)


=======
  optoforce_etherdaq_driver
)

if (abb_rws_interface_FOUND)
  message("--   ABB RWS library found!")

  LIST(APPEND CATKIN_DEPENDS_LIST 
    abb_rws_interface
  )
  LIST(APPEND SOURCEFILES
    src/yumi_hw.cpp
    src/yumi_hw_rws.cpp
  )

  if (abb_egm_interface_FOUND)
    message("--   ABB EGM library found!")
    message("--   All dependencies satisfied. The robot hardware interfaces (via EGM and RWS) will be built")

    LIST(APPEND CATKIN_DEPENDS_LIST 
      abb_egm_interface
    )
    LIST(APPEND SOURCEFILES
      src/yumi_hw_egm.cpp
    )

    add_definitions(-DHAVE_EGM=1)
  else()
    message("--   ABB EGM library not found")
    message("--   Only the robot hardware interface via RWS will be built")
  endif()

  find_package(catkin REQUIRED COMPONENTS ${CATKIN_DEPENDS_LIST})

  add_service_files(
    FILES
    YumiGrasp.srv
  )

  generate_messages(
    DEPENDENCIES
    std_msgs
  )

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS ${CATKIN_DEPENDS_LIST}
  )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

  add_library(${PROJECT_NAME}
    ${SOURCEFILES}
  )

  add_executable(yumi_hw_ifce_node src/yumi_hw_ifce_node.cpp)
  add_executable(yumi_gripper_node src/yumi_gripper_node.cpp)
  add_dependencies(yumi_gripper_node ${PROJECT_NAME}_generate_messages_cpp)

  target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} )
  target_link_libraries( yumi_hw_ifce_node ${catkin_LIBRARIES} ${PROJECT_NAME} simple_message)
  target_link_libraries( yumi_gripper_node ${catkin_LIBRARIES} ${PROJECT_NAME} simple_message)
else()
  message("--   ABB RWS library not found")
  message("--   This package is highly dependent on RWS, and its files cannot be built without it")
endif()
>>>>>>> 3142d96... General improvements in cmake files. Now it is possible to build the package without rws/egm
