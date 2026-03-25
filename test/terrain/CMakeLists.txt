# ---------------------------------------------------------------------------
# liteaero::terrain tests
# Activated in Step 8 (terrain namespace migration).
# ---------------------------------------------------------------------------

set(LAF_TERRAIN_TEST_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/Terrain_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/TerrainTile_test.cpp
)

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
