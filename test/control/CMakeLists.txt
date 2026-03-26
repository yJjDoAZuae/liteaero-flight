# ---------------------------------------------------------------------------
# liteaero::control tests
# Activated in Step 2 (control namespace migration).
# ---------------------------------------------------------------------------

file(GLOB LAF_CONTROL_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*_test.cc"
)

if(NOT LAF_CONTROL_TEST_SOURCES)
    message(STATUS "liteaero_control_test: no test sources found — skipping")
    return()
endif()

add_executable(liteaero_control_test ${LAF_CONTROL_TEST_SOURCES})

target_include_directories(liteaero_control_test PRIVATE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
)

target_compile_features(liteaero_control_test PRIVATE cxx_std_17)

target_link_libraries(liteaero_control_test
    PRIVATE
        liteaero::control
        liteaero_flight_proto
        GTest::gtest_main
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
)

include(GoogleTest)
gtest_discover_tests(liteaero_control_test
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)
