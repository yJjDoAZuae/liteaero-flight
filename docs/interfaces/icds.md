# LiteAero Flight — Interface Control Documents

LiteAero Flight owns the interface definitions for its inputs and outputs. LiteAero Sim
consumes these definitions as a dependency.

Interface types are plain structs with no inheritance or virtual dispatch. They form the
boundary between the flight software subsystems and between liteaero-flight and
liteaero-sim.

---

## ICD Ownership

| ICD | Type / Interface | Owned by | Status |
| --- | --- | --- | --- |
| ICD-F1 | `AircraftCommand` | liteaero-flight | Stub — defined in Step 6 |
| ICD-F2 | `KinematicStateSnapshot` | liteaero-flight | Stub — defined in Step 6 |
| ICD-F3 | `NavigationState` | liteaero-flight | Stub — defined in Step 7 |
| ICD-F4 | Sensor measurement structs (`AirDataMeasurement`, `GnssMeasurement`, `MagMeasurement`) | liteaero-flight | Stub — defined in Step 7 |
| ICD-F5 | `V_Terrain` | liteaero-flight (`liteaero::terrain`) | Header moved in Step 1; namespace applied in Step 4 |
| ICD-F6 | `ILogger` / `LogSource` | liteaero-flight (`liteaero::log`) | Headers moved in Step 1; namespace applied in Step 3 |

---

## ICD Entries

ICD entries are added here as each interface is finalized (Steps 3–8). Each entry
documents the wire layout, units, and coordinate frame.

*No entries yet — populated per step.*
