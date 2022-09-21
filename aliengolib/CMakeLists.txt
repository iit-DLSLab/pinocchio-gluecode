include_directories(
    include
    SYSTEM
    /usr/include/eigen3
    /opt/ros/noetic/include
)

add_library(aliengo 
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


target_link_libraries(aliengo
    robotlib
    roscpp
    urdf
)

install(
    DIRECTORY   include
    DESTINATION /usr/include/
    COMPONENT   aliengo_commons_dev
)

install(
    TARGETS     aliengo
    DESTINATION /usr/lib/robots/
    COMPONENT   aliengo_commons
)

install(
    FILES       aliengo.urdf
    DESTINATION /usr/lib/robots/
	COMPONENT   aliengo_commons
)

# add_subdirectory(tests)