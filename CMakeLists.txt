# Project configuration
cmake_minimum_required(VERSION 3.15)
project(aliengo_lib)

add_compile_options("-std=c++2a")
set(CMAKE_BUILD_TYPE Release)

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

# verify var definition if the building comes from the dls_deploy or is stand_alone
if(NOT DEFINED DLS_INSTALL_MODEL_DIR)
    list(APPEND CMAKE_MODULE_PATH "/usr/include/dls2/cmake")
    include(dls_options)
endif()

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

# set the output name of the library
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "aliengo")

# ================================== Install =================================
install(
    DIRECTORY   include
    DESTINATION ${DLS_INSTALL_MODEL_HEADER_DIR}
    COMPONENT   aliengo_lib_dev
)

install(
    TARGETS     ${PROJECT_NAME}
    DESTINATION ${DLS_INSTALL_MODEL_DIR}
    COMPONENT   aliengo_lib
)

add_subdirectory(tests)

# ================================== Packaging =================================
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
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS           ON)

set(CPACK_DEBIAN_ALIENGO_LIB_PACKAGE_NAME            "dls-aliengo-lib" CACHE INTERNAL "")
set(CPACK_DEBIAN_ALIENGO_LIB_FILE_NAME               "dls-aliengo-lib-${PROJECT_VERSION}.deb" CACHE INTERNAL "")  
set(CPACK_DEBIAN_ALIENGO_LIB_PACKAGE_DEPENDS         "dls-robotlib, dls-aliengo-description" CACHE INTERNAL "")

set(CPACK_DEBIAN_ALIENGO_LIB_DEV_PACKAGE_NAME        "dls-aliengo-lib-dev" CACHE INTERNAL "")
set(CPACK_DEBIAN_ALIENGO_LIB_DEV_FILE_NAME           "dls-aliengo-lib-dev-${PROJECT_VERSION}.deb" CACHE INTERNAL "")  
set(CPACK_DEBIAN_ALIENGO_LIB_DEV_PACKAGE_DEPENDS     "dls-aliengo-lib, dls-robotlib-dev" CACHE INTERNAL "")

get_cmake_property(CPACK_COMPONENTS_ALL COMPONENTS)
list(REMOVE_ITEM CPACK_COMPONENTS_ALL "Unspecified")

include(CPack)