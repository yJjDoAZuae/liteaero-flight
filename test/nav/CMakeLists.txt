# ---------------------------------------------------------------------------
# liteaero::nav tests
# Activated in Step 7 (KinematicStateSnapshot and WGS84 migration).
# ---------------------------------------------------------------------------

set(LAF_NAV_TEST_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/WGS84_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/KinematicStateSnapshot_test.cpp"
)

if(NOT LAF_NAV_TEST_SOURCES)
    message(STATUS "liteaero_nav_test: no test sources found — skipping")
    return()
endif()

add_executable(liteaero_nav_test ${LAF_NAV_TEST_SOURCES})

target_include_directories(liteaero_nav_test PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)

target_compile_features(liteaero_nav_test PRIVATE cxx_std_17)

target_link_libraries(liteaero_nav_test
    PRIVATE
        liteaero::nav
        liteaero_flight_proto
        GTest::gtest_main
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
)

include(GoogleTest)
gtest_discover_tests(liteaero_nav_test
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)
