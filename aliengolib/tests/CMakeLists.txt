add_aliengolib_test(TARGET aliengo_model_test SOURCES aliengo_model_test.cpp)
add_aliengolib_test(TARGET forward_kinematics_unit_tests SOURCES unit_tests/forward_kinematics_unit_tests.cpp)
add_aliengolib_test(TARGET inverse_kinematics_unit_tests SOURCES unit_tests/inverse_kinematics_unit_tests.cpp)
add_aliengolib_test(TARGET inverse_dynamics_unit_tests SOURCES unit_tests/inverse_dynamics_unit_tests.cpp)

get_property(my_list_content GLOBAL PROPERTY ALL_TEST_TARGETS)
add_custom_target(check COMMAND make test DEPENDS "${my_list_content}" COMMENT "Running tests...")