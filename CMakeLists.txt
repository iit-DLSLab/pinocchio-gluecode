# Project configuration
cmake_minimum_required(VERSION 3.15)
project(aliengo)

add_compile_options("-std=c++2a")

# ==============================================================================
# Calculate correct project version number
# ==============================================================================
# Read in the current version
file(READ version.txt PROJECT_VERSION)

# create array of version strings
string(REPLACE "." ";" VERSION_ARRAY ${PROJECT_VERSION})
# ------------------------------------------------------------------------------
# Major, Minor, Patch numbers
# ------------------------------------------------------------------------------
# set corresponding cmake version variables
list(POP_FRONT VERSION_ARRAY
	PROJECT_VERSION_MAJOR
	PROJECT_VERSION_MINOR
	PROJECT_VERSION_PATCH
)
# Remove the trailing newline from the patch number
string(REPLACE "\n" "" PROJECT_VERSION_PATCH ${PROJECT_VERSION_PATCH})

# ------------------------------------------------------------------------------
# Full version number
# ------------------------------------------------------------------------------
# Rebuild the project version including the current commit hash
string(JOIN "." PROJECT_VERSION
	${PROJECT_VERSION_MAJOR}
	${PROJECT_VERSION_MINOR}
	${PROJECT_VERSION_PATCH}
)
unset(VERSION_ARRAY) # delete aray of version strings
message(STATUS "ALIENGO COMMONS PROJECT VERSION ${PROJECT_VERSION}")

# ==============================================================================
# Setup the repository
# ==============================================================================
# This will perfom task such as setting up the git hooks for anyone who uses
# this project
execute_process(
	COMMAND           ./setup.sh
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
)

add_library(${PROJECT_NAME}
    SHARED
        src/aliengo.cpp
        src/aliengo_leg.cpp
        src/utils.cpp
        src/robcogen/transforms.cpp
        src/robcogen/jacobians.cpp
        src/robcogen/inverse_dynamics.cpp
        src/robcogen/inverse_kinematics.cpp
        src/robcogen/inertia_properties.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        include
    PRIVATE
        SYSTEM
        /usr/include/eigen3
)

target_link_directories(${PROJECT_NAME}
    PRIVATE
        /usr/lib/dls2
        /usr/local/lib/dls2
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        robotlib
        roscpp
        urdf    
)

install(
    DIRECTORY   include
    DESTINATION /usr/include/
    COMPONENT   ${PROJECT_NAME}_dev
)

install(
    TARGETS     ${PROJECT_NAME}
    DESTINATION /usr/lib/
    COMPONENT   ${PROJECT_NAME}
)

install(
    FILES       aliengo.urdf
    DESTINATION ${DLS_INSTALL_MODEL_DIR}
	COMPONENT   ${PROJECT_NAME}
)

add_subdirectory(tests)

set(CPACK_PACKAGE_VERSION_MAJOR              ${PROJECT_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR              ${PROJECT_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH              ${PROJECT_VERSION_PATCH})
set(CPACK_DEBIAN_PACKAGE_VERSION             ${PROJECT_VERSION})
set(CPACK_PACKAGE_VENDOR                     "dlslab")
set(CPACK_PACKAGE_CONTACT                    "dlslab")
set(CPACK_DEBIAN_PACKAGE_MAINTAINER          "dlslab")
set(CPACK_VERBATIM_VARIABLES                 ON)
set(CPACK_GENERATOR                          "DEB")
set(CPACK_DEB_COMPONENT_INSTALL              ON)

set(CPACK_DEBIAN_ALIENGO_PACKAGE_NAME            "dls-libaliengo" CACHE INTERNAL "")
set(CPACK_DEBIAN_ALIENGO_FILE_NAME               "dls-libaliengo-${PROJECT_VERSION}.deb" CACHE INTERNAL "")  
set(CPACK_DEBIAN_ALIENGO_PACKAGE_DEPENDS         "robotlib, aliengo-description" CACHE INTERNAL "")

set(CPACK_DEBIAN_ALIENGO_DEV_PACKAGE_NAME        "dls-libaliengo-dev" CACHE INTERNAL "")
set(CPACK_DEBIAN_ALIENGO_DEV_FILE_NAME           "dls-libaliengo-dev-${PROJECT_VERSION}.deb" CACHE INTERNAL "")  
set(CPACK_DEBIAN_ALIENGO_DEV_PACKAGE_DEPENDS     "dls-libaliengo, robotlib-dev" CACHE INTERNAL "")

get_cmake_property(CPACK_COMPONENTS_ALL COMPONENTS)
list(REMOVE_ITEM CPACK_COMPONENTS_ALL "Unspecified")

include(CPack)