cmake_minimum_required(VERSION 2.8.3)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

project(dronemis CXX)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        dynamic_reconfigure
        geometry_msgs
        sensor_msgs
        std_msgs
        std_srvs
        message_generation
        ardrone_autonomy
        cv_bridge

        )

find_package(OpenCV REQUIRED)



## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
        FILES
        Num.msg
)

## Generate services in the 'srv' folder
add_service_files(
        FILES
        AddTwoInts.srv
)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

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
        INCLUDE_DIRS #include
        LIBRARIES dronemis
        CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
        DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(dronemis
#   src/${PROJECT_NAME}/dronemis.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure

#add_executable(controller src/controller.cpp)
#target_link_libraries(controller ${catkin_LIBRARIES})
#add_dependencies(controller dronemis_generate_messages_cpp)

add_executable(controller src/controller.cpp)
target_link_libraries(controller ${catkin_LIBRARIES})
add_dependencies(controller dronemis_generate_messages_cpp)

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})
add_dependencies(talker dronemis_generate_messages_cpp)

add_executable(listener src/listener.cpp src/navdata/Nav.cpp src/navdata/Nav.h)
target_link_libraries(listener ${catkin_LIBRARIES})
add_dependencies(listener dronemis_generate_messages_cpp)

set(BLINDFLIGHT_SOURCE_FILES
        src/flightControl/Route.cpp
        src/flightControl/FlightController.cpp
        src/flightControl/Route.cpp
        src/GUI/ControlPanel/controlpanel.cpp
        src/OpenCv/Color.cpp
        src/OpenCv/Cascade.cpp
        src/OpenCv/CV_Handler.cpp
        src/main.cpp
        src/navdata/Nav.cpp
        src/OpenCv/QR.cpp)

set(BLINDFLIGHT_HEADER_FILES
        src/flightControl/Route.h
        src/flightControl/FlightController.h
        src/flightControl/Command.h
        src/flightControl/Route.h
        src/OpenCv/Color.h
        src/OpenCv/Cascade.h
        src/OpenCv/CV_Handler.h
        src/GUI/ControlPanel/controlpanel.h
        src/navdata/Nav.h
        src/OpenCv/QR.h
        src/Vector3.h
        src/debug.h)

set(BLINDFLIGHT_UI_FILES
        src/GUI/ControlPanel/controlpanel.ui)

set(BLINDFLIGHT_RESOURCE_FILES
        )

# do QT stuff
ADD_DEFINITIONS(-Wall)
find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
QT4_ADD_RESOURCES(BLINDFLIGHT_RESOURCE_FILES_CPP ${BLINDFLIGHT_RESOURCE_FILES})
QT4_WRAP_UI(BLINDFLIGHT_UI_FILES_HPP ${BLINDFLIGHT_UI_FILES})
QT4_WRAP_CPP(BLINDFLIGHT_HEADER_FILES_HPP ${BLINDFLIGHT_HEADER_FILES})
include_directories(${CMAKE_CURRENT_BINARY_DIR})


add_executable(blindFlight src/main.cpp ${BLINDFLIGHT_SOURCE_FILES} ${BLINDFLIGHT_HEADER_FILES} ${BLINDFLIGHT_RESOURCE_FILES_CPP} ${BLINDFLIGHT_UI_FILES_HPP} ${BLINDFLIGHT_HEADER_FILES_HPP})
target_link_libraries(blindFlight ${QT_LIBRARIES} cvd ${catkin_LIBRARIES} zbar ${ZBAR_LIBRARIES})
add_dependencies(blindFlight dronemis_generate_messages_cpp )
target_include_directories(blindFlight PUBLIC  ${ZBAR_INCLUDE_DIRS})

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS dronemis dronemis_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_dronemis.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
