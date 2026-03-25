#pragma once
// liteaero::guidance::PathGuidance — lateral path tracking (outer loop).
//
// Design authority: FC-7 (liteaero-sim/docs/roadmap/flight_code.md)
// Target namespace: liteaero::guidance
// Derives from: liteaero::control::DynamicElement
//
// Implements a nonlinear guidance law (L1 or similar) that commands a target
// heading or roll attitude set point to null cross-track error against a path
// segment. Wraps around liteaero::autopilot::Autopilot.
//
// Not yet implemented. Stub created in Step 9 of the migration plan.

namespace liteaero::guidance {

class PathGuidance;

} // namespace liteaero::guidance
