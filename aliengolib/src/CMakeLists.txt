include_directories(SYSTEM 
    ${roscpp_INCLUDE_DIRS}
)
add_library(aliengolib SHARED
    aliengo.cpp
    aliengo_leg.cpp
    utils.cpp
    robcogen/inertia_properties.cpp
    robcogen/inverse_dynamics.cpp
    robcogen/transforms.cpp
    robcogen/jacobians.cpp
    other/jacobians.cpp
)
 
target_link_libraries(aliengolib
    robotlib
)
install(
    TARGETS 
        aliengolib
    DESTINATION
        /usr/lib/robots/
)

