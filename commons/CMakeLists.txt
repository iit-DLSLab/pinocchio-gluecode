add_library(aliengo_commons
    SHARED 
        # Common code
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
        src/inertia_properties.cpp
)

# Include directories
target_include_directories(aliengo_commons
    PUBLIC
        include 
    PRIVATE
        /usr/include/eigen3
)

target_link_directories(aliengo_commons
    PRIVATE
        /usr/lib/dls2
        /usr/local/lib/dls2
)

target_link_libraries(aliengo_commons
    iitcommons
)


install(
    DIRECTORY   include
    DESTINATION /usr/include
    COMPONENT   aliengo_commons_dev
    PATTERN ".svn" EXCLUDE
)

install(
    TARGETS     aliengo_commons
    DESTINATION /usr/lib
    COMPONENT   aliengo_commons
)