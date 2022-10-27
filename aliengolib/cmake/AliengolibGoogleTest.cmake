find_package(GTest REQUIRED)
include(GoogleTest)

define_property(GLOBAL PROPERTY ALL_TEST_TARGETS
    BRIEF_DOCS "All GTest Targets"
    FULL_DOCS "All GTest Targets")

set_property(GLOBAL PROPERTY ALL_TEST_TARGETS "")

function(add_aliengolib_test)
    include(CMakeParseArguments)
    cmake_parse_arguments(PARSE_ARGV 0 "TEST" "" "TARGET" "SOURCES;LIBRARIES")
    message("Adding Test: ${TEST_TARGET}...")
		add_executable(${TEST_TARGET} ${TEST_SOURCES})
		install(TARGETS ${TEST_TARGET} DESTINATION ${CMAKE_INSTALL_BINDIR})
		target_link_libraries(${TEST_TARGET} ${TEST_LIBRARIES} aliengolib GTest::GTest GTest::Main)
		gtest_discover_tests(${TEST_TARGET})
		set_property(GLOBAL APPEND PROPERTY ALL_TEST_TARGETS "${TEST_TARGET}")
endfunction()

