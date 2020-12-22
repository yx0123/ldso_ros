cmake_minimum_required(VERSION 2.8)
project(ldso_ros)


# Set default build type if not specified otherwise.
# See https://cmake.org/pipermail/cmake/2012-May/050243.html
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  message(STATUS "Setting build type to '${CMAKE_BUILD_TYPE}' as none was specified.")
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

# C++ standard
if(${CMAKE_VERSION} VERSION_LESS "3.2.0")
  set( CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}" )
else()
  set(CMAKE_CXX_STANDARD 11)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  cv_bridge
)

##########################################################################
# You can set DSO_PATH to your source folder by passing
# `-DDSO_PATH=...`, setting environment variables, or by uncommenting
# the following lines. DSO_LIBRARY_PATH defaults to "${DSO_PATH}/build/lib.
#
# If DSO_PATH is not set, we'll try to find a DSO install with cmake
# (which also works in works with DSO installed in the devel folder).
##########################################################################


set(LDSO_PATH         /home/dh/workspaces/ldso_ws/src/LDSO)
set(LDSO_LIBRARY_PATH /home/dh/workspaces/ldso_ws/src/LDSO/lib)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# set values from environment variables
if (NOT LDSO_PATH)
    set(LDSO_PATH $ENV{LDSO_PATH})
endif()
if (NOT LDSO_LIBRARY_PATH)
    set(LDSO_LIBRARY_PATH $ENV{LDSO_LIBRARY_PATH})
    if (NOT LDSO_LIBRARY_PATH AND LDSO_PATH)
        set(LDSO_LIBRARY_PATH "${LDSO_PATH}/build/lib")
    endif()
endif()

if (LDSO_PATH)
    # Manually set path to LDSO source folder
    message("LDSO_PATH set, trying to find library.")
    message("---- EXPECTING LDSO sources at\n\"${LDSO_PATH}\"")
    set(LDSO_INCLUDE_DIRS "${LDSO_PATH}/include" "${LDSO_PATH}/thirdparty")
    message("---- LOOKING FOR LDSO library at\n\"${LDSO_LIBRARY_PATH}\"")
    find_library(LDSO_LIBRARY ldso ${LDSO_LIBRARY_PATH})
else()
    # Detect LDSO install (also catkin devel folder)
    message("LDSO_PATH not set yet, trying to find installed dso headers and library.")
    find_path(LDSO_INCLUDE_DIRS ldso)
    if (LDSO_INCLUDE_DIRS)
        set(LDSO_INCLUDE_DIRS "${LDSO_INCLUDE_DIRS}")
        message("---- FOUND LDSO headers at \"${LDSO_INCLUDE_DIRS}\"")
    endif()
    find_library(LDSO_LIBRARY ldso)
endif()
message("---- Found LDSO include at \"${LDSO_INCLUDE_DIRS}\"")

if (NOT LDSO_INCLUDE_DIRS)
    message(FATAL_ERROR "LDSO headers not found and/or LDSO_PATH not set.")
endif()
if (NOT LDSO_LIBRARY)
    message(FATAL_ERROR "LDSO library not found and/or LDSO_LIBRARY_PATH not set.")
endif()

message("---- Found LDSO library at \"${LDSO_LIBRARY}\"")



find_package(Pangolin 0.2 REQUIRED)
message("\n\n---- FOUND Pangolin Headers at\n\"${Pangolin_INCLUDE_DIRS}\"")
message("---- FOUND Pangolin Libs at\n\"${Pangolin_LIBRARIES}\"")

find_package(OpenCV REQUIRED)
message("\n\n---- FOUND OpenCV Headers at\n\"${OpenCV_INCLUDE_DIRS}\"")
message("---- FOUND OpenCV Libs at\n\"${OpenCV_LIBS}\"")

find_package(Eigen3 REQUIRED)
find_package(Boost COMPONENTS system thread)
find_package(Glog REQUIRED)
message("\n\n---- FOUND Glog Headers at\n\"${GLOG_INCLUDE_DIRS}\"")

catkin_package(
  CATKIN_DEPENDS
    geometry_msgs
    roscpp
    sensor_msgs
    cv_bridge
)

set(CMAKE_CXX_FLAGS "-Wall -Wno-deprecated -march=native -Wno-duplicate-decl-specifier -Wno-ignored-qualifiers -Wno-reorder -Wno-missing-braces")
if(NOT APPLE)
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
endif()
###########
## Build ##
###########


include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${LDSO_INCLUDE_DIRS}
  ${Pangolin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${LDSO_PATH}/thirdparty/g2o/
  ${LDSO_PATH}/thirdparty/DBoW3/

)

set( THIRD_PARTY_LIBS
        ${OpenCV_LIBS}
        ${GLOG_LIBRARY}
        ${Pangolin_LIBRARIES}
        ${LDSO_PATH}/thirdparty/g2o/lib/libg2o${CMAKE_SHARED_LIBRARY_SUFFIX}
        ${LDSO_PATH}/thirdparty/DBoW3/build/src/libDBoW3${CMAKE_SHARED_LIBRARY_SUFFIX}
)

## Declare a C++ executable
add_executable(ldso_live src/main.cpp)

#add_subdirectory(src)


target_link_libraries(ldso_live
    ${LDSO_LIBRARY}
    ${catkin_LIBRARIES}
    ${GLOG_LIBRARY}
    ${THIRD_PARTY_LIBS}
)
