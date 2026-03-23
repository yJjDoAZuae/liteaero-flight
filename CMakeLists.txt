cmake_minimum_required(VERSION 3.16)
project(LiteAeroFlight LANGUAGES CXX)

# ---------------------------------------------------------------------------
# Project-wide settings
# ---------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Prevent CMake from using @response-files for include paths in compile_commands.json,
# which VSCode IntelliSense cannot expand.
set(CMAKE_CXX_USE_RESPONSE_FILE_FOR_INCLUDES OFF)

option(BUILD_SHARED_LIBS "Build libraries as shared instead of static" OFF)

# Enable CTest at the project root so all subdirectories are registered.
enable_testing()

# ---------------------------------------------------------------------------
# Dependencies
#
# C++ dependencies are managed via Conan 2.x. Run:
#   conan install . --output-folder=build --build=missing
# then configure with:
#   cmake -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake
#
# See docs/dependencies/README.md and docs/installation/README.md.
#
# Registry:
#   Dependency       | Version         | License      | Method
#   -----------------|-----------------|--------------|----------------------
#   Eigen3           | 3.4.0           | MPL-2        | Conan (find_package)
#   nlohmann_json    | 3.12.0          | MIT          | Conan (find_package)
#   GTest            | 1.14.0          | BSD-3-Clause | Conan (find_package)
#   protobuf         | 3.21.12         | BSD-3-Clause | Conan (find_package)
#   mcap             | v1.4.0          | MIT          | FetchContent (header-only, pattern 1b)
# ---------------------------------------------------------------------------

# --- Eigen3 (MPL-2) — linear algebra ---
find_package(Eigen3 REQUIRED NO_MODULE)

# --- nlohmann/json (MIT) — JSON serialization ---
find_package(nlohmann_json REQUIRED)

# --- Google Test (BSD-3-Clause) — unit testing ---
find_package(GTest REQUIRED)

# --- protobuf (BSD-3-Clause) — binary serialization ---
# v3.21.x is the last pre-abseil release; avoids pulling in ~30 abseil targets.
find_package(protobuf REQUIRED CONFIG)

# --- mcap (MIT) — MCAP binary log format; C++ header-with-implementation library ---
# Pattern 1b: only the include path is needed; MCAP_IMPLEMENTATION is defined in
# src/log/mcap_impl.cpp. Compression disabled at compile time (no lz4/zstd deps).
include(FetchContent)
FetchContent_Declare(
    mcap
    GIT_REPOSITORY https://github.com/foxglove/mcap.git
    GIT_TAG        releases/cpp/v1.4.0
)
FetchContent_GetProperties(mcap)
if(NOT mcap_POPULATED)
    FetchContent_Populate(mcap)
    add_library(mcap_headers INTERFACE)
    target_include_directories(mcap_headers SYSTEM INTERFACE
        ${mcap_SOURCE_DIR}/cpp/mcap/include
    )
    target_compile_definitions(mcap_headers INTERFACE
        MCAP_COMPRESSION_NO_LZ4
        MCAP_COMPRESSION_NO_ZSTD
    )
endif()

# ---------------------------------------------------------------------------
# Subdirectories
# ---------------------------------------------------------------------------
add_subdirectory(proto)
add_subdirectory(src)

# Tests are only built when liteaero-flight is the top-level CMake project.
# When consumed via add_subdirectory (e.g., from liteaero-sim during Phase 1
# migration), tests are skipped — each step's verification runs liteaero-flight
# tests standalone and liteaero-sim tests in the sim build separately.
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    add_subdirectory(test)
endif()
