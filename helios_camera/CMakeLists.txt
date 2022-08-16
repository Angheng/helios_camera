cmake_minimum_required(VERSION 3.0.2)
project(helios_camera)

find_package(arena_sdk QUIET)
if (NOT ${arena_sdk_FOUND})
    include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/Findarena_sdk.cmake")
endif()
# CATKIN
#


add_definitions("-std=gnu++11")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} [OpenCV.cmake PATH])

set(CATKIN_COMPONENTS
     camera_control_msgs # arena_camera depends on this upper custom msg package 
     camera_info_manager
     cv_bridge
     image_geometry
     image_transport
     roscpp
     roslaunch 
     sensor_msgs
)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
    catkin 
        REQUIRED
    COMPONENTS 
        ${CATKIN_COMPONENTS}
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIR})

catkin_package(
    # INCLUDE_DIRS
    #     include
    LIBRARIES
        ${PROJECT_NAME}
    CATKIN_DEPENDS
        ${CATKIN_COMPONENTS} # package dep for build/run
    # DEPENDS # sys dep for build/run 
    #    <> 
    #    <> 
)

set(helios_camera helios_camera)

add_executable(
    ${helios_camera}  # runs by rosrun
        src/helios_camera.cpp
)

target_include_directories(
    ${helios_camera}
        PRIVATE ${catkin_INCLUDE_DIRS} 
        PRIVATE ${arena_sdk_INCLUDE_DIRS}
        PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# needed ".so"s
target_link_libraries(
    ${helios_camera}
        ${catkin_LIBRARIES}
        ${arena_sdk_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

add_dependencies(
    ${helios_camera}
        # because this uses one of the catkin componenets which is camera_control_msg
        ${catkin_EXPORTED_TARGETS} 
)

set(pcl_to_image pcl_to_image)

add_executable(
    ${pcl_to_image}  # runs by rosrun
        src/pcl_to_image.cpp
)

target_include_directories(
    ${pcl_to_image}
        PRIVATE ${catkin_INCLUDE_DIRS}
)

# needed ".so"s
target_link_libraries(
    ${pcl_to_image}
       ${catkin_LIBRARIES}
       ${OpenCV_LIBRARIES}
)

install(
    DIRECTORY
        launch/
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    FILES_MATCHING PATTERN "*.launch"
)

install(
    DIRECTORY
        config/
    DESTINATION
        ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    FILES_MATCHING PATTERN "*.yaml"
)

install(
    TARGETS
        ${helios_camera}
        ${pcl_to_image}
    LIBRARY DESTINATION
        ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
    DIRECTORY
        include/${PROJECT_NAME}/
    DESTINATION
        ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN
        "*.h"
    PATTERN "internal" EXCLUDE
)
