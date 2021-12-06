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

add_executable(inverse_dynamics_unit_tests
	unit_tests/inverse_dynamics_unit_tests.cpp
)

target_link_libraries(inverse_dynamics_unit_tests
	PRIVATE
	robotlib
	${CMAKE_DL_LIBS}
	gtest
)