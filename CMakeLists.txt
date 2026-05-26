# Project configuration
cmake_minimum_required(VERSION 3.15)
project(glue_code)

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
message(STATUS "GLUE_CODE COMMONS PROJECT VERSION ${PROJECT_VERSION}")

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
include(dls_options RESULT_VARIABLE DLS_OPTIONS_INCLUDED OPTIONAL)
if (NOT DLS_OPTIONS_INCLUDED)
    message(STATUS "${PROJECT_NAME}: Trying to include dls_options from /usr/include/dls2/cmake")
    list(APPEND CMAKE_MODULE_PATH "/usr/include/dls2/cmake")
    include(dls_options RESULT_VARIABLE DLS_OPTIONS_INCLUDED OPTIONAL)
    if(NOT DLS_OPTIONS_INCLUDED)
        set (DLS_INSTALL_MODEL_HEADER_DIR "/usr/include/dls2/robots")
        set (DLS_INSTALL_MODEL_DIR "/usr/lib/dls2/robots")
        message(STATUS "${PROJECT_NAME}: Could not include dls_options.cmake from dls2 installation. Setting default installation paths: ${DLS_INSTALL_MODEL_HEADER_DIR}, ${DLS_INSTALL_MODEL_DIR}")
    else()
        message(STATUS "${PROJECT_NAME}: Successfully included dls_options.cmake from dls2 installation.")
    endif()
else()
    message(STATUS "${PROJECT_NAME}: Successfully included dls_options.cmake from local cmake.")
endif()

set(COAL_DISABLE_HPP_FCL_WARNINGS ON)
find_package(pinocchio REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(${PROJECT_NAME}
    SHARED
        src/glue_code.cpp
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
        include
        ${pinocchio_INCLUDE_DIRS}
)

target_link_directories(${PROJECT_NAME}
    PRIVATE
        /usr/lib/dls2
        /usr/local/lib/dls2
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Eigen3::Eigen
        robotlib
        urdfdom_model
        urdfdom_model_state
        urdfdom_world
        urdfdom_sensor
        ${pinocchio_LIBRARIES}
        yaml-cpp   
)

# set the output name of the library
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "glue_code")

# ================================== Install =================================
install(
    DIRECTORY   include
    DESTINATION ${DLS_INSTALL_MODEL_HEADER_DIR}
    COMPONENT   glue_code_dev
)

install(
    TARGETS     ${PROJECT_NAME}
    DESTINATION ${DLS_INSTALL_MODEL_DIR}
    COMPONENT   glue_code
)

# add_subdirectory(tests)

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

set(CPACK_DEBIAN_GLUE_CODE_PACKAGE_NAME            "dls-glue_code-lib" CACHE INTERNAL "")
set(CPACK_DEBIAN_GLUE_CODE_FILE_NAME               "dls-glue_code-lib-${PROJECT_VERSION}.deb" CACHE INTERNAL "")  
set(CPACK_DEBIAN_GLUE_CODE_PACKAGE_DEPENDS         "dls-robotlib, dls-glue_code-description" CACHE INTERNAL "")

set(CPACK_DEBIAN_GLUE_CODE_DEV_PACKAGE_NAME        "dls-glue_code-lib-dev" CACHE INTERNAL "")
set(CPACK_DEBIAN_GLUE_CODE_DEV_FILE_NAME           "dls-glue_code-lib-dev-${PROJECT_VERSION}.deb" CACHE INTERNAL "")  
set(CPACK_DEBIAN_GLUE_CODE_DEV_PACKAGE_DEPENDS     "dls-glue_code-lib, dls-robotlib-dev" CACHE INTERNAL "")

get_cmake_property(CPACK_COMPONENTS_ALL COMPONENTS)
list(REMOVE_ITEM CPACK_COMPONENTS_ALL "Unspecified")

include(CPack)