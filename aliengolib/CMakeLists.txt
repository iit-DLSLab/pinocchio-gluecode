cmake_minimum_required(VERSION 3.8.0)
project(aliengolib)

option(BUILD_TESTING "" OFF)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
include(CTest)

find_package(robotlib REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(urdf REQUIRED)
find_library(URDFLIB urdf)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CSS_STANDARD_REQUIRED True)

add_compile_options(
	-std=c++17
	-fPIC
	$<$<CONFIG:Debug>:-g>  #-g produces debugging information, it is enabled if Debug configuration is set
	-Wall -Wextra -pedantic # Enable warnings
	-O3
)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/include
	SYSTEM
	/usr/include/eigen3
	/opt/ros/noetic/include
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src)

if (BUILD_TESTING)
	enable_testing()
	include(AliengolibGoogleTest)
	add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/tests)
endif()

install(
    FILES
        aliengo.urdf
    DESTINATION
        /usr/lib/robots
)