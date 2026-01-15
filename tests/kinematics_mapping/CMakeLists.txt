enable_testing()

add_compile_options("-std=c++20")

# compile test
add_executable(kinematics_mapping_test kinematics_mapping_test.cpp)
target_link_libraries(kinematics_mapping_test PRIVATE yaml-cpp)