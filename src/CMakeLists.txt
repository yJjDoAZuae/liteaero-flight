# ---------------------------------------------------------------------------
# LiteAero Flight — subsystem library targets
#
# One CMake target per C++ namespace. Targets are INTERFACE at this stage
# (Step 1 — directory layout only). Sources and link dependencies are added
# per subsystem in Steps 2–8 as namespace changes are applied.
#
# Target mapping:
#   liteaero_log      / liteaero::log      — ILogger, logging sinks
#   liteaero_control  / liteaero::control  — DynamicElement, SisoElement, Filter hierarchy
#   liteaero_terrain  / liteaero::terrain  — terrain mesh types, V_Terrain
#   liteaero_nav      / liteaero::nav      — WGS84, NavigationFilter, estimators
#
# All targets expose include/ as a public include directory so consumers
# can locate headers before the namespace migration is complete.
# ---------------------------------------------------------------------------

# --- liteaero::log ---
add_library(liteaero_log INTERFACE)
add_library(liteaero::log ALIAS liteaero_log)
target_include_directories(liteaero_log INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(liteaero_log INTERFACE cxx_std_17)

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
