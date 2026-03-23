# ---------------------------------------------------------------------------
# LiteAero Flight — subsystem library targets
#
# One CMake target per C++ namespace.
#
# Target mapping:
#   liteaero_log      / liteaero::log      — ILogger, logging sinks
#   liteaero_control  / liteaero::control  — DynamicElement, SisoElement, Filter hierarchy
#   liteaero_terrain  / liteaero::terrain  — terrain mesh types, V_Terrain
#   liteaero_nav      / liteaero::nav      — WGS84, NavigationFilter, estimators
# ---------------------------------------------------------------------------

# --- liteaero::log ---
add_library(liteaero_log STATIC
    log/Logger.cpp
    log/LogReader.cpp
    log/mcap_impl.cpp
)
add_library(liteaero::log ALIAS liteaero_log)
target_include_directories(liteaero_log
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)
target_compile_features(liteaero_log PUBLIC cxx_std_17)
target_link_libraries(liteaero_log
    PUBLIC
        nlohmann_json::nlohmann_json
    PRIVATE
        liteaero_flight_proto
        protobuf::libprotobuf
        mcap_headers
)

# --- liteaero::control ---
add_library(liteaero_control INTERFACE)
add_library(liteaero::control ALIAS liteaero_control)
target_include_directories(liteaero_control INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(liteaero_control INTERFACE cxx_std_17)

# --- liteaero::terrain ---
add_library(liteaero_terrain INTERFACE)
add_library(liteaero::terrain ALIAS liteaero_terrain)
target_include_directories(liteaero_terrain INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(liteaero_terrain INTERFACE cxx_std_17)

# --- liteaero::nav ---
add_library(liteaero_nav INTERFACE)
add_library(liteaero::nav ALIAS liteaero_nav)
target_include_directories(liteaero_nav INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(liteaero_nav INTERFACE cxx_std_17)
