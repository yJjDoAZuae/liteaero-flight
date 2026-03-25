#pragma once
// liteaero::nav::NavigationFilter — fuses GNSS, air-data, and magnetometer.
//
// Design authority: FC-8 / docs/architecture/navigation_filter.md (not yet created)
// Target namespace: liteaero::nav
// Derives from: liteaero::control::DynamicElement
//
// Depends on: SensorGnss, SensorAirData, SensorMag (not yet defined).
// Output: liteaero::nav::KinematicStateSnapshot (or NavigationState).
//
// Not yet implemented. Stub created in Step 9 of the migration plan.

namespace liteaero::nav {

class NavigationFilter;

} // namespace liteaero::nav
