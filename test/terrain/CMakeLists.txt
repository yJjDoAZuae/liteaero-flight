# ---------------------------------------------------------------------------
# liteaero::terrain tests
# Activated in Step 4 (terrain namespace migration).
# ---------------------------------------------------------------------------

# Terrain tests are not yet ready to build — TerrainTile and Terrain headers
# have not been migrated to liteaero-flight. Add sources here in the terrain
# migration step.
set(LAF_TERRAIN_TEST_SOURCES)

if(NOT LAF_TERRAIN_TEST_SOURCES)
    message(STATUS "liteaero_terrain_test: no test sources found — skipping")
    return()
endif()

add_executable(liteaero_terrain_test ${LAF_TERRAIN_TEST_SOURCES})

target_include_directories(liteaero_terrain_test PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
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
