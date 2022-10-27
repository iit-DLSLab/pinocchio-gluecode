include_directories(SYSTEM 
    ${roscpp_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
    aliengo.cpp
    aliengo_leg.cpp
    utils.cpp
    robcogen/transforms.cpp
    robcogen/jacobians.cpp
    robcogen/inverse_dynamics.cpp
    robcogen/inverse_kinematics.cpp
    robcogen/inertia_properties.cpp
)


target_link_libraries(${PROJECT_NAME}
    robotlib
    ${URDFLIB}
)

install(
    TARGETS 
        aliengolib
    DESTINATION
        /usr/lib/robots/${PROJECT_NAME}/
)