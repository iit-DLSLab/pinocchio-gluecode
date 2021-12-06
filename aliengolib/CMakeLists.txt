cmake_minimum_required(VERSION 3.5.1)
project(aliengolib)

set(MAJOR 0)
set(MINOR 0)
set(PATCH 0)

# TODO: Understand the command below
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

find_package(robotlib REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

add_compile_options(
	-std=c++17
	-fPIC
	$<$<CONFIG:Debug>:-g>
#	-Wall -Wextra -pedantic # Enable warnings
#	-fsanitize=undefined -g -O3	# UBSAN - Undefined Behaviour Sanitizer
)
#add_link_options(-fsanitize=undefined) # UBSAN - Undefined Behaviour Sanitizer

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/include/robcogen
	SYSTEM
	/usr/include/eigen3
	#${CMAKE_CURRENT_SOURCE_DIR}/iit-commons/include
)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/tests)