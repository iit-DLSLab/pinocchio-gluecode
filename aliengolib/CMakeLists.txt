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

target_include_directories(aliengo
    PUBLIC
        include
    PRIVATE
        SYSTEM
        /usr/include/eigen3
)

target_link_directories(aliengo
    PRIVATE
        /usr/lib/dls2
        /usr/local/lib/dls2
)

target_link_libraries(aliengo
    PUBLIC
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
    DESTINATION /usr/lib/
    COMPONENT   aliengo_commons
)

install(
    FILES       aliengo.urdf
    DESTINATION /usr/include/robots/
	COMPONENT   aliengo_commons
)

# add_subdirectory(tests)