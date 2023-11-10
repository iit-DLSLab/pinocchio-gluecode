enable_testing()

add_executable(aliengo_model_test
    aliengo_model_test.cpp
)

target_link_libraries(aliengo_model_test
	PRIVATE
		aliengo_lib
		robotlib
		${CMAKE_DL_LIBS}
		gtest
)

target_include_directories(aliengo_model_test
    PUBLIC
        include
    PRIVATE
        /usr/include/eigen3
)

add_executable(forward_kinematics_unit_tests
	unit_tests/forward_kinematics_unit_tests.cpp
)

target_link_libraries(forward_kinematics_unit_tests
	PRIVATE
		aliengo_lib
		robotlib
		${CMAKE_DL_LIBS}
		gtest
)

target_include_directories(forward_kinematics_unit_tests
    PUBLIC
        include
    PRIVATE
        /usr/include/eigen3
)

add_executable(inverse_kinematics_unit_tests
	unit_tests/inverse_kinematics_unit_tests.cpp
)

target_link_libraries(inverse_kinematics_unit_tests
	PRIVATE
		aliengo_lib
		robotlib
		${CMAKE_DL_LIBS}
		gtest
)

target_include_directories(inverse_kinematics_unit_tests
    PUBLIC
        include
    PRIVATE
        /usr/include/eigen3
)

add_executable(inverse_dynamics_unit_tests
	unit_tests/inverse_dynamics_unit_tests.cpp
)

target_link_libraries(inverse_dynamics_unit_tests
	PRIVATE
		aliengo_lib
		robotlib
		${CMAKE_DL_LIBS}
		gtest
)

target_include_directories(inverse_dynamics_unit_tests
    PUBLIC
        include
    PRIVATE
        /usr/include/eigen3
)

add_executable(read_urdf_test
	other_tests/read_urdf_test.cpp
)

target_link_libraries(read_urdf_test
	PRIVATE
		${CMAKE_DL_LIBS}
		gtest
)

target_include_directories(read_urdf_test
    PUBLIC
        include
    PRIVATE
        /usr/include/eigen3
)