cmake_minimum_required(VERSION 3.0.2)
project(mission_sequencer)

## Determine CMAKE options
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release")
endif()


## Compile with highest possible c++ standard
# http://stackoverflow.com/a/25836953
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++17" COMPILER_SUPPORTS_CXX17)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX17)
    set(CMAKE_CXX_STANDARD 17)
elseif(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++0x support. Please use a different C++ compiler.")
endif()

## Set CMAKE compile flags
if (CMAKE_BUILD_TYPE STREQUAL "Release")
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fomit-frame-pointer -fno-signed-zeros")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fsee -fomit-frame-pointer -fno-signed-zeros -fno-math-errno -funroll-loops -Wno-dev")
elseif (CMAKE_BUILD_TYPE STREQUAL "Debug")
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3 -Wall -Wuninitialized -Wmaybe-uninitialized -fno-omit-frame-pointer")
endif ()

message("Build Type: " ${CMAKE_BUILD_TYPE})
message(WARNING "CMAKE_CXX_FLAGES: ${CMAKE_CXX_FLAGS}")

#####
# COSTUM CMAKE
#####

# Include custom cmake modules
include(cmake/Custom.cmake)
include(cmake/GetGitRevisionDescription.cmake)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  mavros_msgs
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  MissionRequest.msg
  MissionResponse.msg
  MissionWaypoint.msg
  MissionWaypointArray.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GetStartPose.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES amaze_mission_sequencer
 CATKIN_DEPENDS roscpp rospy std_msgs mavros_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Setup header and source paths
set(include_path "${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}")
set(source_path  "${CMAKE_CURRENT_SOURCE_DIR}/src")

## Group files

set(headers
    ${include_path}/types/sequencer_types.hpp
    ${include_path}/types/sequencer_options.hpp
    ${include_path}/types/sequencer_waypoint.hpp
    ${include_path}/utils/message_conversion.hpp
    ${include_path}/utils/parser_ros.hpp
    ${include_path}/utils/parser_waypoints.hpp
    ${include_path}/mission_sequencer.hpp
)

set(sources
    ${source_path}/mission_sequencer_node.cpp
    ${source_path}/mission_sequencer.cpp
    ${source_path}/parser_waypoints.cpp
)

## Group Source and Header files correctly
set(header_group "Header Files (API)")
set(source_group "Source Files")
source_group_by_path(${include_path} "\\\\.h$|\\\\.hpp$"
    ${header_group} ${headers})
source_group_by_path(${source_path}  "\\\\.cpp$|\\\\.c$|\\\\.h$|\\\\.hpp$"
    ${source_group} ${sources})

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${include_path}
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/amaze-mission-sequencer.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node ${headers} ${sources})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

###################
## Documentation ##
###################

# Get git revision
get_git_head_revision(GIT_REFSPEC GIT_SHA1)
string(SUBSTRING "${GIT_SHA1}" 0 12 GIT_REV)
if(NOT GIT_SHA1)
    set(GIT_REV "0")
endif()

# Meta information about the project
set(META_PROJECT_NAME        "mission_sequencer")
set(META_PROJECT_DESCRIPTION "ROS mission sequencer for PX4")
set(META_AUTHOR_ORGANIZATION "University Klagenfurt: CNS")
set(META_AUTHOR_DOMAIN       "https://gitlab.aau.at/aau-cns/ros_pkgs/amaze_mission_sequencer")
set(META_AUTHOR_MAINTAINER   "martin.scheiber@aau.at")
set(META_VERSION_MAJOR       "0")
set(META_VERSION_MINOR       "1")
set(META_VERSION_PATCH       "1")
set(META_VERSION_REVISION    "${GIT_REV}")
set(META_VERSION             "${META_VERSION_MAJOR}.${META_VERSION_MINOR}.${META_VERSION_PATCH}")
set(META_NAME_VERSION        "${META_PROJECT_NAME} v${META_VERSION} (${META_VERSION_REVISION})")
set(META_CMAKE_INIT_SHA      "${GIT_REV}")


option(OPTION_BUILD_DOCS     "Build documentation." ON)
add_subdirectory(docs)
