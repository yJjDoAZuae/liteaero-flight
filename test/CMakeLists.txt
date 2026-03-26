# ---------------------------------------------------------------------------
# LiteAero Flight — test subdirectories
#
# Test executables are defined per subsystem. Each subdirectory activates
# when its subsystem's namespace migration is complete (Steps 2–8).
# enable_testing() is called at the project root.
# ---------------------------------------------------------------------------

add_subdirectory(log)
add_subdirectory(control)
add_subdirectory(terrain)
add_subdirectory(nav)
