# ---------------------------------------------------------------------------
# liteaero::terrain tests
# Activated in Step 4 (terrain namespace migration).
# ---------------------------------------------------------------------------

file(GLOB LAF_TERRAIN_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*_test.cc"
)

if(NOT LAF_TERRAIN_TEST_SOURCES)
    message(STATUS "liteaero_terrain_test: no test sources found — skipping")
    return()
endif()

add_executable(liteaero_terrain_test ${LAF_TERRAIN_TEST_SOURCES})

target_include_directories(liteaero_terrain_test PRIVATE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
)

target_compile_features(liteaero_terrain_test PRIVATE cxx_std_17)

target_link_libraries(liteaero_terrain_test
    PRIVATE
        liteaero::terrain
        liteaero_flight_proto
        GTest::gtest_main
        Eigen3::Eigen
        nlohmann_json::nlohmann_json
)

include(GoogleTest)
gtest_discover_tests(liteaero_terrain_test
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)
