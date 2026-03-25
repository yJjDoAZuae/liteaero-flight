#pragma once
// liteaero::guidance::Path — ordered sequence of V_PathSegment objects.
//
// Design authority: FC-6 (liteaero-sim/docs/roadmap/flight_code.md)
// Target namespace: liteaero::guidance
//
// Path::query() delegates to the active segment and advances to the next
// segment when the current one is complete.
//
// Not yet implemented. Stub created in Step 9 of the migration plan.

namespace liteaero::guidance {

class Path;

} // namespace liteaero::guidance
