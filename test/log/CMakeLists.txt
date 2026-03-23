# ---------------------------------------------------------------------------
# liteaero::log tests
# ---------------------------------------------------------------------------

file(GLOB LAF_LOG_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*_test.cc"
)

if(NOT LAF_LOG_TEST_SOURCES)
    message(STATUS "liteaero_log_test: no test sources found — skipping")
    return()
endif()

add_executable(liteaero_log_test ${LAF_LOG_TEST_SOURCES})

target_include_directories(liteaero_log_test PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

target_compile_features(liteaero_log_test PRIVATE cxx_std_17)

target_link_libraries(liteaero_log_test
    PRIVATE
        liteaero::log
        liteaero_flight_proto
        GTest::gtest_main
        nlohmann_json::nlohmann_json
)

include(GoogleTest)
gtest_discover_tests(liteaero_log_test
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)
