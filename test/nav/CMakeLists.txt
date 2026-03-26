# ---------------------------------------------------------------------------
# liteaero::nav tests
# Activated in Step 5 (nav namespace migration).
# ---------------------------------------------------------------------------

file(GLOB LAF_NAV_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*_test.cc"
)

if(NOT LAF_NAV_TEST_SOURCES)
    message(STATUS "liteaero_nav_test: no test sources found — skipping")
    return()
endif()

add_executable(liteaero_nav_test ${LAF_NAV_TEST_SOURCES})

target_include_directories(liteaero_nav_test PRIVATE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
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
