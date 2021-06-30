# Project configuration
cmake_minimum_required(VERSION 2.8)
project(aliengo_commons)

SET(CMAKE_CXX_FLAGS "-std=c++11")

#set(HEADER_INSTALL_ROOT /usr/local/include/ CACHE PATH "Where to install the headers")
#set(LIB_INSTALL_ROOT /usr/local/lib/ CACHE PATH "Where to install the shared library")
find_package(PkgConfig)
pkg_check_modules(EIGEN3 eigen3 REQUIRED)

find_package(catkin REQUIRED iit_commons)

# Include directories
include_directories(include ${EIGEN3_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

set(LIB_NAME iitaliengocommons)

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS iit_commons
   LIBRARIES ${LIB_NAME}
   DEPENDS
)

# Source code folder
set(SOURCES # Common code
            src/internals.cpp
            src/joints_pid_control.cpp
            src/hydraulic_dynamics.cpp
            src/hydraulics_parameters.cpp
            src/feet_contact_forces.cpp
            src/robot_limits.cpp
            # Generated code
            src/transforms.cpp
            src/jacobians.cpp
            src/jsim.cpp
            src/inverse_dynamics.cpp
            src/inverse_kinematics.cpp
            src/forward_dynamics.cpp
            src/inertia_properties.cpp)

# Add the shared library
add_library(${LIB_NAME} SHARED ${SOURCES})
target_link_libraries(${LIB_NAME} ${catkin_LIBRARIES})

#if (CATKIN_ENABLE_TESTING)
#    find_package(catkin REQUIRED COMPONENTS rostest)
#    include_directories(include ${catkin_INCLUDE_DIRS})

    #catkin_add_gtest(fbl-test test/fbl.cpp)
    #target_link_libraries(fbl-test ${LIB_NAME} ${catkin_LIBRARIES} pthread)
#endif()

install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(TARGETS ${LIB_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
