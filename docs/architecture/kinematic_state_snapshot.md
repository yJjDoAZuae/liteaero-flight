# KinematicStateSnapshot — Design Authority

This document defines the data architecture for `KinematicStateSnapshot` and
`GeodeticPosition`, the value types that replace `KinematicState` and `WGS84_Datum` at the
`liteaero-flight` / `liteaero-sim` interface. It also resolves the **shared interface target
name** (Step 6 open question) and specifies the utility function organization.

This document is authoritative for Step 7 implementation. No code is written in this step.

---

## Problem Statement

`KinematicState` and `WGS84_Datum` in liteaero-sim conflate two distinct concerns:

| Concern | Current location | Problem |
| --- | --- | --- |
| **Data** — the minimal set of values that uniquely and non-redundantly represents the state | Mixed into `KinematicState` and `WGS84_Datum` fields | Cannot be passed across the library boundary as a plain value; tightly coupled to the simulation integrator |
| **Derived quantities** — values computable from the data (Euler angles, frame-transformed velocities, radii of curvature) | Methods on `KinematicState` and `WGS84_Datum` | Repeated or re-implemented wherever these types are consumed; must travel with the class, not usable without it |
| **Simulation engine** — the `step()` integrator and state-construction logic | `KinematicState` constructors and `step()` | Sim-specific; must not leak into flight software |

The goal is to separate these three concerns cleanly.

---

## Design Principles

1. **Value types carry data only.** `KinematicStateSnapshot` and `GeodeticPosition` are plain
   structs — public fields, no methods beyond construction. They are trivially copyable and
   serializable, and carry no ownership of external resources.

2. **Derived quantities live in utility namespaces.** Functions that compute derived quantities
   from a snapshot are free functions in a utility namespace, not methods on the data type.
   This gives one authoritative implementation usable by both flight software and the
   simulator, without requiring the data type to carry the computation.

3. **Non-redundant storage.** Only values that cannot be derived from the other stored values
   are stored. See the redundancy analysis below.

4. **No Euler-angle singularities in storage.** Orientation is stored as quaternions.
   Euler angles are computable as needed via utility functions; singularities at pitch ±90°
   are confined to those utility functions and do not affect reconstruction.

5. **One namespace, one CMake target for all shared value types.** `KinematicStateSnapshot`,
   `GeodeticPosition`, `AircraftCommand`, `NavigationState`, and sensor measurement structs
   all live in `liteaero::nav` and the `liteaero::nav` CMake target. A dedicated interface
   target is not needed; `liteaero::nav` already appears on the namespace map and is the
   natural home for navigation state types.

---

## Shared Interface Target Decision

The Step 6 open question asked for a CMake target name and C++ namespace for
`KinematicStateSnapshot`, `AircraftCommand`, `NavigationState`, and sensor measurement
structs.

**Decision: `liteaero::nav` / CMake target `liteaero::nav`.**

Rationale: `WGS84` was already planned for `liteaero::nav`. All shared state types
(`KinematicStateSnapshot`, `GeodeticPosition`, `NavigationState`, `AircraftCommand`) describe
navigation-domain quantities. Aggregating them into the existing `liteaero::nav` target avoids
adding an extra CMake target with no other content. The `liteaero::nav` stub target in
`src/CMakeLists.txt` will be promoted from INTERFACE to STATIC when Step 7 is implemented.

Update the Key Design Decisions table in this plan accordingly.

---

## `GeodeticPosition`

### Current state

`WGS84_Datum` stores three values — geodetic latitude, longitude, height — but also carries
20+ methods: radius-of-curvature computations, ECEF/NED conversions, gravity, Earth rotation.

### Proposed split

```cpp
// include/liteaero/nav/GeodeticPosition.hpp
namespace liteaero::nav {

/// Minimal WGS84 geodetic position — data only.
/// All derived quantities (radii of curvature, ECEF, gravity) are in namespace WGS84.
struct GeodeticPosition {
    double latitude_rad  = 0.0;   ///< Geodetic latitude, radians
    double longitude_rad = 0.0;   ///< Longitude, radians
    float  altitude_m    = 0.0f;  ///< Height above WGS84 ellipsoid, meters
};

} // namespace liteaero::nav
```

Latitude and longitude are `double` because single-precision float loses ~1 m resolution at
mid-latitudes. Altitude is `float` because 1 cm altitude resolution is more than sufficient.

### WGS84 utility namespace

```cpp
// include/liteaero/nav/WGS84.hpp
namespace liteaero::nav::WGS84 {

// Ellipsoid constants
inline constexpr double kA_m       = 6378137.0;          ///< Semi-major axis, m
inline constexpr double kFinv      = 298.257223563;       ///< Inverse flattening
inline constexpr double kGM        = 3.986004418e14;      ///< Gravitational constant
inline constexpr double kOmega_rps = 7.2921151467e-5;     ///< Earth rotation rate, rad/s

// Derived from above — computed at compile time
// ...

double meridionalRadius(double latitude_rad);
double primeVerticalRadius(double latitude_rad);
double northRadius(const GeodeticPosition& p);
double eastRadius(const GeodeticPosition& p);
double latitudeRate_rad_s(const GeodeticPosition& p, float v_north_mps);
double longitudeRate_rad_s(const GeodeticPosition& p, float v_east_mps);
Eigen::Vector3d transportRate(const GeodeticPosition& p, float v_north_mps, float v_east_mps);
double gravity_mps2(const GeodeticPosition& p);
Eigen::Vector3d omega_ie_n(const GeodeticPosition& p);
Eigen::Vector3d toECEF(const GeodeticPosition& p);
GeodeticPosition fromECEF(const Eigen::Vector3d& ecef);
Eigen::Quaterniond qne(const GeodeticPosition& p);
GeodeticPosition fromQne(const Eigen::Quaterniond& qne);

} // namespace liteaero::nav::WGS84
```

`WGS84_Datum` in liteaero-sim is redesigned in Step 7 to hold a `GeodeticPosition` and
delegate all computational methods to `WGS84::` free functions. All call sites remain
unchanged through Step 7 — the sim-side `WGS84_Datum` wrapper preserves the existing API
during the transition.

---

## `KinematicStateSnapshot`

### Redundancy analysis

`KinematicState` stores 13 primary fields and 2 mutable cached fields. The 2 cached fields
(`_pom`, `_turn_circle`) are derived from velocity and acceleration and are not stored in the
snapshot. Of the 13 primary fields, one is redundant:

| Field | Role | Decision |
| --- | --- | --- |
| `_q_nw` | Wind-to-NED rotation | **Store** — primary orientation |
| `_q_nb` | Body-to-NED rotation | **Derived** — `q_nb = q_nw · Ry(α) · Rz(−β)` by the aerodynamic angle definition; not stored |
| `_alpha_rad` | Angle of attack | **Store** |
| `_beta_rad` | Sideslip angle | **Store** |
| All others | No derivable relationship | **Store** |

`q_nb` is excluded from the snapshot because it is always exactly reconstructible from
`q_nw`, `alpha_rad`, and `beta_rad` — this is definitional, not an approximation. Storing
both would be redundant by 4 floats (16 bytes).

### Struct definition

```cpp
// include/liteaero/nav/KinematicStateSnapshot.hpp
#pragma once
#include <liteaero/nav/GeodeticPosition.hpp>
#include <Eigen/Geometry>

namespace liteaero::nav {

/// Minimal non-redundant kinematic state.
///
/// All stored values are in SI units. Orientation is stored as quaternions to avoid
/// Euler-angle singularities. q_nb is not stored because it is derivable from
/// q_nw, alpha_rad, and beta_rad — see KinematicStateUtil::q_nb().
///
/// Derived quantities (Euler angles, frame-transformed velocities, radii of
/// curvature, turn circle, plane of motion) are computed by free functions in
/// namespace KinematicStateUtil.
struct KinematicStateSnapshot {

    // ── Time ────────────────────────────────────────────────────────────────
    double time_s = 0.0;           ///< Simulation time, seconds

    // ── Position ────────────────────────────────────────────────────────────
    GeodeticPosition position;     ///< WGS84 geodetic position (lat/lon/alt)

    // ── Translational state ─────────────────────────────────────────────────
    Eigen::Vector3f velocity_ned_mps       = Eigen::Vector3f::Zero();
    Eigen::Vector3f acceleration_ned_mps   = Eigen::Vector3f::Zero();

    // ── Orientation ─────────────────────────────────────────────────────────
    /// Wind-to-NED rotation. q_nb is derived via KinematicStateUtil::q_nb().
    Eigen::Quaternionf q_nw = Eigen::Quaternionf::Identity();

    // ── Angular rates ───────────────────────────────────────────────────────
    Eigen::Vector3f rates_body_rps = Eigen::Vector3f::Zero();

    // ── Aerodynamic angles and their rates ──────────────────────────────────
    float alpha_rad          = 0.0f;   ///< Angle of attack, radians
    float beta_rad           = 0.0f;   ///< Sideslip angle, radians
    float alpha_dot_rad_s    = 0.0f;   ///< Angle of attack rate, rad/s
    float beta_dot_rad_s     = 0.0f;   ///< Sideslip angle rate, rad/s
    float roll_rate_wind_rad_s = 0.0f; ///< Roll rate of Wind frame w.r.t. NED, rad/s

    // ── Wind ────────────────────────────────────────────────────────────────
    Eigen::Vector3f wind_ned_mps = Eigen::Vector3f::Zero();
};

} // namespace liteaero::nav
```

### Size summary

| Field | Type | Bytes |
| --- | --- | --- |
| `time_s` | `double` | 8 |
| `position` (`GeodeticPosition`) | 2× `double` + `float` | 20 |
| `velocity_ned_mps` | `Vector3f` | 12 |
| `acceleration_ned_mps` | `Vector3f` | 12 |
| `q_nw` | `Quaternionf` | 16 |
| `rates_body_rps` | `Vector3f` | 12 |
| `alpha_rad`, `beta_rad`, `alpha_dot_rad_s`, `beta_dot_rad_s`, `roll_rate_wind_rad_s` | 5× `float` | 20 |
| `wind_ned_mps` | `Vector3f` | 12 |
| **Total** | | **112 bytes** |

The original `KinematicState` stored 13 fields including `q_nb` (16 bytes extra) and the
`WGS84_Datum` object (which carries method vtable overhead in the current sim implementation).
The snapshot is smaller and layout-stable.

---

## `KinematicStateUtil`

All derived quantities from `KinematicStateSnapshot` are free functions in
`namespace liteaero::nav::KinematicStateUtil`. This is the single authoritative implementation
— both the sim's `KinematicState` and any flight-software consumer call the same functions.

```cpp
// include/liteaero/nav/KinematicStateUtil.hpp
namespace liteaero::nav::KinematicStateUtil {

// ── Orientation ──────────────────────────────────────────────────────────────

/// Body-to-NED rotation. Derived from q_nw, alpha, beta.
/// q_nb = q_nw · Ry(alpha) · Rz(-beta)
Eigen::Quaternionf q_nb(const KinematicStateSnapshot& s);

/// Stability-to-NED rotation: q_ns = q_nw · Ry(alpha)
Eigen::Quaternionf q_ns(const KinematicStateSnapshot& s);

/// NED-to-Local-Level rotation (from position datum).
Eigen::Quaternionf q_nl(const KinematicStateSnapshot& s);


// ── Euler angles (derived; subject to singularity at pitch = ±90°) ────────────
/// Roll angle, radians. Extracted from q_nb via ZYX decomposition.
float roll_rad(const KinematicStateSnapshot& s);

/// Pitch angle, radians. Singularity at ±π/2.
float pitch_rad(const KinematicStateSnapshot& s);

/// Heading/yaw angle, radians.
float heading_rad(const KinematicStateSnapshot& s);

/// Euler rates [roll_rate, pitch_rate, heading_rate], rad/s.
/// Returns zero vector when pitch is within 1e-6 rad of ±π/2 (gimbal lock guard).
Eigen::Vector3f euler_rates_rad_s(const KinematicStateSnapshot& s);


// ── Frame-transformed velocities ──────────────────────────────────────────────

/// Velocity in body frame, m/s.
Eigen::Vector3f velocity_body_mps(const KinematicStateSnapshot& s);

/// Wind-relative velocity in wind frame, m/s.
Eigen::Vector3f velocity_wind_mps(const KinematicStateSnapshot& s);

/// Airspeed magnitude, m/s.
float airspeed_mps(const KinematicStateSnapshot& s);


// ── Frame-transformed accelerations ──────────────────────────────────────────

/// Acceleration in body frame, m/s².
Eigen::Vector3f acceleration_body_mps2(const KinematicStateSnapshot& s);

/// Acceleration in wind frame, m/s².
Eigen::Vector3f acceleration_wind_mps2(const KinematicStateSnapshot& s);


// ── Geometry ──────────────────────────────────────────────────────────────────

/// Crab angle (ground-track to wind-relative velocity), radians.
float crab_rad(const KinematicStateSnapshot& s);

/// Crab angle rate, rad/s.
float crab_rate_rad_s(const KinematicStateSnapshot& s);

/// Plane of motion from velocity/acceleration curvature.
PlaneOfMotion plane_of_motion(const KinematicStateSnapshot& s);

/// Turn radius and center from centripetal acceleration.
TurnCircle turn_circle(const KinematicStateSnapshot& s);

} // namespace liteaero::nav::KinematicStateUtil
```

---

## `KinematicState` Post-Migration (sim-only)

`KinematicState` in liteaero-sim is redesigned in Step 7 as follows:

```cpp
// liteaero-sim: include/KinematicState.hpp (post-Step-7)
namespace liteaerosim {

class KinematicState {
public:
    // ── Simulation engine ────────────────────────────────────────────────────
    // Two constructors (matching current behaviour — see KinematicState.cpp)
    KinematicState(double time_s, liteaero::nav::GeodeticPosition position,
                   Eigen::Vector3f velocity_ned_mps,
                   Eigen::Vector3f acceleration_ned_mps,
                   Eigen::Quaternionf q_nb,
                   Eigen::Vector3f rates_body_rps);

    KinematicState(double time_s, liteaero::nav::GeodeticPosition position,
                   Eigen::Vector3f velocity_ned_mps,
                   Eigen::Vector3f acceleration_wind_mps,
                   Eigen::Quaternionf q_nw,
                   float roll_rate_wind_rad_s,
                   float alpha_rad, float beta_rad,
                   float alpha_dot_rad_s, float beta_dot_rad_s,
                   Eigen::Vector3f wind_ned_mps);

    void step(Eigen::Vector3f forces_n, Eigen::Vector3f moments_nm,
              float dt_s);

    // ── Snapshot access ──────────────────────────────────────────────────────
    const liteaero::nav::KinematicStateSnapshot& snapshot() const { return snapshot_; }

    // ── Derived quantity pass-throughs (delegate to KinematicStateUtil) ──────
    // These forward to KinematicStateUtil free functions so that all call sites
    // that currently use state.roll(), state.heading(), etc. continue to compile
    // without modification during the transition. They are not re-implementations.
    float roll_rad()        const;
    float pitch_rad()       const;
    float heading_rad()     const;
    // ... (one thin forwarding wrapper per existing KinematicState method)

    // ── Direct field access (mirrors current KinematicState API) ─────────────
    double                          time_s()              const;
    liteaero::nav::GeodeticPosition position()            const;
    Eigen::Vector3f                 velocity_ned_mps()    const;
    // ...

private:
    liteaero::nav::KinematicStateSnapshot snapshot_;
};

} // namespace liteaerosim
```

**Key properties of this design:**

- `KinematicState` owns a `KinematicStateSnapshot` by composition. It is not a subclass.
  Composition is preferred because `KinematicState` IS NOT a snapshot — it is a simulation
  element that produces snapshots.
- Derived quantity methods on `KinematicState` are **one-line forwarding wrappers** to
  `KinematicStateUtil` free functions. There is no duplicated logic.
- Control loop signatures change from `const KinematicState&` to
  `const liteaero::nav::KinematicStateSnapshot&` in Step 7. The sim passes
  `state.snapshot()`. Flight software passes a snapshot directly.
- `WGS84_Datum` in liteaero-sim is redesigned to hold a `GeodeticPosition` and delegate all
  methods to `WGS84::` free functions, preserving the existing sim call sites.

---

## Files to Create in Step 7

| File | Target | Contents |
| --- | --- | --- |
| `include/liteaero/nav/GeodeticPosition.hpp` | `liteaero::nav` | Value struct — lat, lon, alt |
| `include/liteaero/nav/WGS84.hpp` | `liteaero::nav` | Free functions: radii, gravity, ECEF, qne |
| `src/nav/WGS84.cpp` | `liteaero::nav` | Implementations (ported from `WGS84.cpp`) |
| `include/liteaero/nav/KinematicStateSnapshot.hpp` | `liteaero::nav` | Value struct — 14 fields |
| `include/liteaero/nav/KinematicStateUtil.hpp` | `liteaero::nav` | Free function declarations |
| `src/nav/KinematicStateUtil.cpp` | `liteaero::nav` | Implementations (ported from `KinematicState.cpp`) |
| `test/nav/WGS84_test.cpp` | `liteaero::nav` | Update existing stub |
| `test/nav/KinematicStateSnapshot_test.cpp` | `liteaero::nav` | New: construction, round-trip, derived quantities |

The `liteaero_nav` CMake target in `src/CMakeLists.txt` is promoted from INTERFACE to STATIC
and linked against `Eigen3::Eigen` and `nlohmann_json::nlohmann_json`.

---

## Open Questions Resolved

| Question (from Step 6) | Resolution |
| --- | --- |
| Shared interface target name | `liteaero::nav` / `liteaero::nav` CMake target |
| Store `q_nb` in snapshot? | No — derived from `q_nw · Ry(α) · Rz(−β)` |
| `WGS84_Datum` in snapshot? | No — replaced by `GeodeticPosition` value struct; methods move to `WGS84::` namespace |
| Derived quantities on snapshot? | No — all in `KinematicStateUtil::` free functions |
| `KinematicState` post-migration: inheritance or composition? | Composition — `KinematicState` owns a `KinematicStateSnapshot`; forwarding wrappers preserve call sites |
| `POM` and `TurnCircle` in snapshot? | No — computed on demand by `KinematicStateUtil::plane_of_motion()` and `turn_circle()` |
| `control_subsystem_access` to derived quantities | Control loops receive `const KinematicStateSnapshot&`; call `KinematicStateUtil::` functions directly |
