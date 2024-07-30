enable_testing()

add_compile_options("-std=c++20")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include(dls_message)		# message generation function

set(GAZEBO_GLUE_CODE_TEST gazebo_gluecode_test)

add_library(${GAZEBO_GLUE_CODE_TEST} 
    SHARED 
        src/gazebo_glue_code_test.cpp
)

set_property(TARGET ${GAZEBO_GLUE_CODE_TEST} PROPERTY POSITION_INDEPENDENT_CODE ON)

target_link_directories(${GAZEBO_GLUE_CODE_TEST}
    PUBLIC
        /usr/lib/dls2
        /usr/local/lib/dls2
)

include_directories(
    /usr/include/eigen3
    /usr/include/ignition/msgs5
    /usr/include/ignition/math6
    /usr/include/ignition/fuel_tools4
    /usr/include/ignition/transport8
    /usr/include/ignition/cmake2
    /usr/include/ignition/common3
    /usr/local/include/gazebo-11
    /usr/local/include/sdformat-9.10
)

target_include_directories(${GAZEBO_GLUE_CODE_TEST}
    PUBLIC
        include
        ${GAZEBO_INCLUDE_DIRS}
        ${CMAKE_CURRENT_BINARY_DIR}/include
)

target_link_libraries(${GAZEBO_GLUE_CODE_TEST}
    PRIVATE
        ${GAZEBO_LIBRARIES}
        dls_framework
        robotlib
)

# Add idls here
dls_add_message(gazebo_glue_code_test ${GAZEBO_GLUE_CODE_TEST}) 	# generate message

#-- Install target

install(
    TARGETS     ${GAZEBO_GLUE_CODE_TEST}
    DESTINATION ${DLS_INSTALL_HARDWARE_DIR}/gazebo_plugins
    COMPONENT   gazebo_sim
)
