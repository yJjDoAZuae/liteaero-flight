# ---------------------------------------------------------------------------
# liteaero::control tests
# Activated in Step 2 (control namespace migration).
# ---------------------------------------------------------------------------

# Step 4 tests only — Step 5 tests (Integrator, Derivative, RateLimit, Unwrap,
# Antiwindup, Gain, SISOPIDFF, RectilinearTable, TableAxis) are in this directory
# but are not listed here until their headers are migrated in Step 5.
set(LAF_CONTROL_TEST_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/FilterFIR_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/FilterRealizations_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/FilterSS2_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/FilterSS2Clip_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/FilterSS_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/FilterTF_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/FilterTF2_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Limit_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Antiwindup_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Integrator_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Derivative_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/RateLimit_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Unwrap_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/Gain_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/SISOPIDFF_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/RectilinearTable_test.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/TableAxis_test.cpp"
)

if(NOT LAF_CONTROL_TEST_SOURCES)
    message(STATUS "liteaero_control_test: no test sources found — skipping")
    return()
endif()

add_executable(liteaero_control_test ${LAF_CONTROL_TEST_SOURCES})

target_include_directories(liteaero_control_test PRIVATE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
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
