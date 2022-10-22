# Project configuration
cmake_minimum_required(VERSION 3.15)
project(aliengo_commons)

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

add_subdirectory(commons)
add_subdirectory(aliengolib)

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

set(CPACK_DEBIAN_ALIENGO_COMMONS_PACKAGE_NAME            "dls-aliengo-commons" CACHE INTERNAL "")
set(CPACK_DEBIAN_ALIENGO_COMMONS_FILE_NAME               "dls-aliengo-commons-${PROJECT_VERSION}.deb" CACHE INTERNAL "")  
set(CPACK_DEBIAN_ALIENGO_COMMONS_PACKAGE_DEPENDS         "robotlib" CACHE INTERNAL "")

set(CPACK_DEBIAN_ALIENGO_COMMONS_DEV_PACKAGE_NAME        "dls-aliengo-commons-dev" CACHE INTERNAL "")
set(CPACK_DEBIAN_ALIENGO_COMMONS_DEV_FILE_NAME           "dls-aliengo-commons-dev-${PROJECT_VERSION}.deb" CACHE INTERNAL "")  
set(CPACK_DEBIAN_ALIENGO_COMMONS_DEV_PACKAGE_DEPENDS     "dls-aliengo-commons, robotlib-dev" CACHE INTERNAL "")

get_cmake_property(CPACK_COMPONENTS_ALL COMPONENTS)
list(REMOVE_ITEM CPACK_COMPONENTS_ALL "Unspecified")

include(CPack)