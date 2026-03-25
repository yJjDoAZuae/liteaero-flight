# LiteAero Flight — Interface Control Documents

LiteAero Flight owns the interface definitions for its inputs and outputs. LiteAero Sim
consumes these definitions as a dependency.

Interface types are plain structs with no inheritance or virtual dispatch. They form the
boundary between the flight software subsystems and between liteaero-flight and
liteaero-sim.

---

## ICD Ownership

| ICD | Type / Interface | Owned by | Location |
| --- | --- | --- | --- |
| ICD-F1 | `AircraftCommand` | liteaero-flight | `liteaero-sim/include/Aircraft.hpp` — migration to liteaero-flight pending (FC-5) |
| ICD-F2 | `KinematicStateSnapshot` | liteaero-flight | `liteaero-flight/include/liteaero/nav/KinematicStateSnapshot.hpp` |
| ICD-F3 | `NavigationState` | liteaero-flight | Not yet defined — pending FC-8 |
| ICD-F4 | `AirDataMeasurement` | liteaero-flight | `liteaero-sim/include/sensor/SensorAirData.hpp` — migration to liteaero-flight pending (FC-8) |
| ICD-F5 | `V_Terrain` | liteaero-flight | `liteaero-flight/include/liteaero/terrain/V_Terrain.hpp` |
| ICD-F6 | `ILogger` / `LogSource` | liteaero-flight | `liteaero-flight/include/liteaero/log/` |

---

## ICD-F1 — AircraftCommand

**Owner:** liteaero-flight. Type is currently defined in `liteaero-sim/include/Aircraft.hpp`
(`namespace liteaero::simulation`) pending migration to liteaero-flight at FC-5.

**Producer:** Guidance (FC-7) / Autopilot (FC-5); test harness during development.

**Consumer:** `Aircraft::step()`

**Transport:** Direct function-call argument (in-process).

**Content:**

| Field | Unit | Description |
| --- | --- | --- |
| `n_z` | g | Commanded normal load factor |
| `n_y` | g | Commanded lateral load factor |
| `rollRate_Wind_rps` | rad/s | Commanded wind-frame roll rate |
| `throttle_nd` | nd | Normalized throttle [0, 1] |

**Constraints:**

- All values SI. No unit conversion inside `Aircraft`.
- `n_z = 1` is 1 g (level, unaccelerated flight). `n_z = 0` is zero g (free fall).
- `throttle_nd` is clamped to [0, 1] inside `Aircraft`; the caller need not pre-clamp.
- Load factor rate terms for alpha-dot and beta-dot feed-forward are computed internally
  by `Aircraft` via an IIR filtered differentiator; they are not part of this interface.

---

## ICD-F2 — KinematicStateSnapshot

**Owner:** liteaero-flight. Defined in
`liteaero-flight/include/liteaero/nav/KinematicStateSnapshot.hpp`
(`namespace liteaero::nav`).

**Producer:** `KinematicState::step()` in liteaero-sim (populates internal snapshot);
`KinematicState::snapshot()` accessor returns the value.

**Consumers:** All sensors; Logger; future Autopilot, Guidance, and Navigation components.

**Transport:** Const reference or value copy.

**Content:**

| Group | Fields | Unit |
| --- | --- | --- |
| Position | `position` (`GeodeticPosition`: `latitude_rad`, `longitude_rad`, `altitude_m`) | rad, rad, m |
| NED velocity | `velocity_ned_mps` (3-vector) | m/s |
| Attitude | `q_nw` (quaternion, NED→wind frame) | — |
| Body rates | `body_rates_rps` (p, q, r) | rad/s |
| Wind-frame rates | `wind_rates_rps` (roll, pitch, yaw in wind frame) | rad/s |
| Body acceleration | `acceleration_body_mps2` (3-vector) | m/s² |
| NED acceleration | `acceleration_ned_mps2` (3-vector) | m/s² |
| Aerodynamic angles | `alpha_rad`, `beta_rad` | rad |
| Airspeed | `Va_mps` | m/s |

**Constraints:**

- 14 stored fields; `q_nb` is excluded (derived as `q_nw · Ry(α) · Rz(−β)`).
- Euler angles and plane-of-motion are not stored; derived via `KinematicStateUtil`.
- All values SI. Body rates are body-frame; Euler angles use 3-2-1 sequence.

---

## ICD-F3 — NavigationState

**Owner:** liteaero-flight. Not yet defined — pending FC-8 (`NavigationFilter` design).

---

## ICD-F4 — AirDataMeasurement

**Owner:** liteaero-flight. Type is currently defined in
`liteaero-sim/include/sensor/SensorAirData.hpp` (`namespace liteaero::simulation`)
pending migration to liteaero-flight at FC-8.

**Producer:** `SensorAirData::step()`

**Consumers:** Logger; future `NavigationFilter` (FC-8); future Autopilot (FC-5).

**Transport:** Return value from `step()`.

**Content:**

| Field | Unit | Description |
| --- | --- | --- |
| `ias_mps` | m/s | Indicated airspeed (incompressible Bernoulli, ρ₀ reference) |
| `cas_mps` | m/s | Calibrated airspeed (isentropic, ρ₀ reference) |
| `eas_mps` | m/s | Equivalent airspeed (dynamic pressure equivalent at ρ₀) |
| `tas_mps` | m/s | True airspeed |
| `mach_nd` | nd | Mach number |
| `baro_altitude_m` | m | Barometric altitude (Kollsman-referenced) |
| `oat_k` | K | Outside air temperature |

**Constraints:**

- All values include configured noise, lag, and crossflow pressure error.
- `ias_mps`, `cas_mps`, `eas_mps`, `tas_mps`, `mach_nd` are clamped to ≥ 0.

---

## ICD-F5 — Terrain Query Interface (`V_Terrain`)

**Owner:** liteaero-flight. Defined in
`liteaero-flight/include/liteaero/terrain/V_Terrain.hpp`
(`namespace liteaero::terrain`).

**Producer:** `FlatTerrain` or `TerrainMesh`

**Consumers:** Future `SensorRadAlt`, `SensorLaserAlt`; future landing gear contact model;
future guidance (terrain-following, obstacle avoidance).

**Transport:** Virtual function call.

**Interface:**

```cpp
namespace liteaero::terrain {

class V_Terrain {
public:
    virtual float elevation_m(double latitude_rad, double longitude_rad) const = 0;
    float heightAboveGround_m(float altitude_m,
                              double latitude_rad,
                              double longitude_rad) const;
};

} // namespace liteaero::terrain
```

**Constraints:**

- `elevation_m` returns WGS84 ellipsoidal height (approximately MSL for low altitudes).
- `heightAboveGround_m` returns `max(0, altitude_m − elevation_m(lat, lon))`.
- Both `FlatTerrain` and `TerrainMesh` are conforming implementations.

---

## ICD-F6 — Logger Write Interface (`ILogger` / `LogSource`)

**Owner:** liteaero-flight. Defined in `liteaero-flight/include/liteaero/log/`
(`namespace liteaero::log`).

**Producer:** All domain components (via `LogSource`).

**Consumer:** `Logger`

**Transport:** Method call on `LogSource` member.

**Constraints:**

- All logged quantities must be in SI units.
- Channel names are strings; units are encoded in the name where not obvious.
- The `Logger` is write-only during simulation; reading uses `LogReader`.
