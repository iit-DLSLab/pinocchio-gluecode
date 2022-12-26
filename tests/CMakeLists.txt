enable_testing()

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/../include
)

add_executable(aliengo_model_test
    aliengo_model_test.cpp
)

target_link_libraries(aliengo_model_test
	aliengo
	robotlib
	${CMAKE_DL_LIBS}
	gtest
)

add_executable(forward_kinematics_unit_tests
	unit_tests/forward_kinematics_unit_tests.cpp
)

target_link_libraries(forward_kinematics_unit_tests
	PRIVATE
	aliengo
	robotlib
	${CMAKE_DL_LIBS}
	gtest
)

add_executable(inverse_kinematics_unit_tests
	unit_tests/inverse_kinematics_unit_tests.cpp
)

target_link_libraries(inverse_kinematics_unit_tests
	PRIVATE
	aliengo
	robotlib
	${CMAKE_DL_LIBS}
	gtest
)

add_executable(inverse_dynamics_unit_tests
	unit_tests/inverse_dynamics_unit_tests.cpp
)

target_link_libraries(inverse_dynamics_unit_tests
	PRIVATE
	aliengo
	robotlib
	${CMAKE_DL_LIBS}
	gtest
)

add_executable(read_urdf_test
	other_tests/read_urdf_test.cpp
)

target_link_libraries(read_urdf_test
	PRIVATE
	${CMAKE_DL_LIBS}
	gtest
)