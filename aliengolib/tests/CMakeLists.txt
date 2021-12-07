enable_testing()

add_executable(main_tests
	main_tests.cpp
    	aliengo_model_test.cpp
)

target_link_libraries(main_tests
	aliengolib
	robotlib
	${CMAKE_DL_LIBS}
	gtest
)

add_executable(forward_kinematics_unit_tests
	unit_tests/forward_kinematics_unit_tests.cpp
)

target_link_libraries(forward_kinematics_unit_tests
	PRIVATE
	aliengolib
	robotlib
	${CMAKE_DL_LIBS}
	gtest
)

add_executable(inverse_dynamics_unit_tests
	unit_tests/inverse_dynamics_unit_tests.cpp
)

target_link_libraries(inverse_dynamics_unit_tests
	PRIVATE
	robotlib
	${CMAKE_DL_LIBS}
	gtest
)

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/../include/pugixml
)

add_executable(read_urdf_test
	other_tests/read_urdf_test.cpp
	../src/pugixml/pugixml.cpp
)

target_link_libraries(read_urdf_test
	PRIVATE
	${CMAKE_DL_LIBS}
	gtest
)