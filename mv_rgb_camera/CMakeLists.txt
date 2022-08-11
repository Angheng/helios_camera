cmake_minimum_required(VERSION 3.0.2)
project(mv_rgb_camera)

add_definitions("-std=gnu++11")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_COMPONENTS
     cv_bridge
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

set(mv_rgb_camera mv_rgb_camera)

add_executable(
    ${mv_rgb_camera}  # runs by rosrun
        src/mv_rgb_camera.cpp
)

target_include_directories(
    ${mv_rgb_camera}
        PRIVATE ${catkin_INCLUDE_DIRS}
)

# needed ".so"s
target_link_libraries(
    ${mv_rgb_camera}
        ${catkin_LIBRARIES}
        MVSDK
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
        ${mv_rgb_camera}
    LIBRARY DESTINATION
        ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION
        ${CATKIN_PACKAGE_BIN_DESTINATION}
)