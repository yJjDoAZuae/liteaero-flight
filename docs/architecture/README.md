# Architecture

System and subsystem architecture documents for LiteAero Flight.

| Document | Contents |
| --- | --- |
| [dynamic_element.md](dynamic_element.md) | **Design authority** for `DynamicElement` and `SisoElement` — NVI pattern, lifecycle contract, serialization contract, logging interface |
| [liteaero-sim/docs/architecture/dynamic_element.md](../../liteaero-sim/docs/architecture/dynamic_element.md) | Full class hierarchy — Filter hierarchy, Propulsion, control sub-elements (sim-side components pending migration) |
| [liteaero-sim/docs/architecture/logger.md](../../liteaero-sim/docs/architecture/logger.md) | **Design authority** for the `Logger` subsystem — `ILogger`, `Logger`, `LogSource`, `LogReader`, MCAP format, multi-source data model |

Additional architecture documents are added as subsystems are designed and implemented.
Documents for subsystems not yet migrated to liteaero-flight remain in
[liteaero-sim/docs/architecture/](../../liteaero-sim/docs/architecture/) until their
migration step is complete.
