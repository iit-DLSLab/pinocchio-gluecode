enable_testing()

add_compile_options("-std=c++20")

# compile test
add_executable(load_robot_test load_robot.cpp)
target_link_libraries(load_robot_test PRIVATE yaml-cpp robotlib)