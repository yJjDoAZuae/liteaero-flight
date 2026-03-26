# C++ Coding Guidelines

Refer to [general.md](general.md) for project-wide standards on TDD, naming, SI units, serialization, and architecture. This document covers C++-specific conventions.

---

## Language Standard

- **C++17** minimum. Use C++20 features where the toolchain supports them (concepts, ranges, `std::span`).
- Compile with warnings enabled and treated as errors: `-Wall -Wextra -Wpedantic -Werror`.
- Enable sanitizers in debug/test builds: `-fsanitize=address,undefined`.

---

## Naming Conventions (C++)

| Category | Convention | Example |
| --- | --- | --- |
| Classes / Structs | `PascalCase` | `KinematicStateSnapshot`, `RollController` |
| Abstract base classes | `PascalCase`, no prefix | `DynamicElement`, `SisoElement` |
| Methods | `camelCase` | `computeLoadFactor()`, `step()` |
| Private / protected members | `snake_case_` (trailing underscore) | `roll_rate_rad_s_`, `mass_kg_` |
| Public struct fields | `snake_case` (no trailing underscore) | `altitude_m`, `roll_rad` |
| Method parameters | `snake_case` (no trailing underscore) | `dt_s`, `load_factor` |
| Local variables | `snake_case` | `dt_s`, `lift_n` |
| Constants / `constexpr` | `SCREAMING_SNAKE_CASE` | `GRAVITY_MPS2`, `MAX_BANK_RAD` |
| Enums | `enum class`, `PascalCase` type, `PascalCase` values | `AutopilotMode::LateralNav` |
| Namespaces | `snake_case`, lowercase | `namespace liteaero::control`, `namespace liteaero::guidance` |
| Template parameters | `PascalCase` | `template <typename StateType>` |
| Macros | `SCREAMING_SNAKE_CASE` with project prefix | `LAF_ASSERT(...)` |

The trailing underscore on private/protected members is the primary visual signal that a
name is instance state, not a local or parameter. It must be applied consistently: every
`private` and `protected` data member gets it; public struct fields and all function
parameters do not.

```cpp
struct WindConfig {
    float speed_mps     = 0.0f;   // public field — no trailing underscore
    float altitude_m    = 0.0f;
};

class Integrator : public SisoElement {
public:
    void resetTo(float value);    // parameter — no trailing underscore
private:
    float dt_s_   = 0.0f;        // private member — trailing underscore
    float output_ = 0.0f;
};
```

### Unit Encoding in Names

When units are not obvious from context, encode them in the variable name:

```cpp
double altitude_m_;          // member: altitude in meters
double roll_rate_rad_s_;     // member: roll rate in rad/s
double thrust_n_;            // member: thrust in newtons
double bank_angle_rad;       // local: bank angle in radians
constexpr double GRAVITY_MPS2 = 9.80665;
```

---

## File and Directory Structure

```text
include/
  liteaero/
    <subsystem>/
      ClassName.hpp          // public interface
src/
  <subsystem>/
    ClassName.cpp            // implementation
test/
  <subsystem>/
    ClassName_test.cpp       // unit tests
docs/
  guidelines/
```

- One class per header/source pair.
- Headers use `#pragma once`.
- Implementation files include their own header first, then standard library, then third-party, then project headers — each group separated by a blank line.

```cpp
#pragma once

// ClassName.hpp
#include <cstddef>
#include <string>

#include <nlohmann/json.hpp>

#include "liteaero/control/DynamicElement.hpp"
```

---

## Object Lifecycle Interface

Every dynamic flight software component implements this interface:

```cpp
namespace liteaero::control {

class DynamicElement {
public:
    virtual ~DynamicElement() = default;

    /// Initialize from configuration. Called once before first use.
    virtual void initialize(const nlohmann::json& config) = 0;

    /// Restore component to initial post-initialize conditions.
    virtual void reset() = 0;

    /// Advance internal state by one timestep (dt fixed at initialize time).
    virtual void step(float u) = 0;

    /// Return a complete snapshot of internal state (SI units throughout).
    virtual nlohmann::json serializeJson() const = 0;

    /// Restore internal state from a snapshot produced by serializeJson().
    virtual void deserializeJson(const nlohmann::json& state) = 0;
};

} // namespace liteaero::control
```

See [docs/architecture/dynamic_element.md](../architecture/dynamic_element.md) for the full NVI design.

---

## Memory Management

- Use **RAII** for all resource management. Never use raw `new`/`delete`.
- Prefer **value semantics** and stack allocation for small, fixed-size objects.
- Use `std::unique_ptr` for single ownership; `std::shared_ptr` only when shared ownership is genuinely required.
- Prefer `std::vector`, `std::array`, and standard containers over raw arrays.
- Avoid `std::shared_ptr` cycles; use `std::weak_ptr` to break them.

---

## Type Safety and Modern C++

- Prefer `enum class` over plain `enum` for all enumerations.
- Use `constexpr` for all compile-time constants instead of `#define`.
- Use `[[nodiscard]]` on functions whose return value must not be discarded.
- Use `explicit` on single-argument constructors and conversion operators.
- Prefer `auto` for type deduction where the type is obvious from context; avoid it when it obscures the type.
- Use structured bindings and range-for where they improve clarity.
- Avoid raw pointers in interfaces; use references or smart pointers.

```cpp
// Good
[[nodiscard]] double computeLoadFactor(double lift_n, double weight_n) const;

// Bad
double computeLoadFactor(double lift, double weight);  // units unclear
```

---

## SI Units Enforcement

- All function parameters and return values use SI units.
- Unit conversions are isolated in a dedicated `unit_conversion` module/header.
- Never call unit conversion functions inside physics or control computation code.

```cpp
// Domain code: pure SI
double computeBankAngle(double lateral_accel_mps2, double speed_mps) const;

// Interface/config code: convert at the boundary
double bank_rad = units::deg_to_rad(config.at("bank_angle_deg").get<double>());
```

---

## Serialization

Use [nlohmann/json](https://github.com/nlohmann/json) as the standard JSON library.

### Pattern

```cpp
nlohmann::json Integrator::serializeJson() const {
    return {
        {"schema_version", 1},
        {"output_", output_},
        {"dt_s_", dt_s_}
    };
}

void Integrator::deserializeJson(const nlohmann::json& j) {
    const int version = j.at("schema_version").get<int>();
    if (version != 1) {
        throw std::runtime_error("Integrator: unsupported schema version");
    }
    output_ = j.at("output_").get<float>();
    dt_s_   = j.at("dt_s_").get<float>();
}
```

### Rules

- All serialized field names use SI unit suffixes: `"altitude_m"`, `"roll_rate_rad_s"`.
- Schema version is always field `"schema_version"` (integer).
- Round-trip test is mandatory for every serializable class.

---

## Testing (C++)

### Framework

Use **Google Test (gtest)** with **Google Mock (gmock)** for mocking dependencies.

### Test File Structure

```cpp
// test/control/Integrator_test.cpp
#include <gtest/gtest.h>
#include "liteaero/control/Integrator.hpp"

namespace liteaero::control {
namespace {

class IntegratorTest : public ::testing::Test {
protected:
    void SetUp() override {
        integrator_.initialize(default_config_);
    }

    Integrator integrator_;
    nlohmann::json default_config_ = {{"dt_s", 0.01}, {"initial_value", 0.0}};
};

TEST_F(IntegratorTest, ZeroInputProducesZeroOutput) {
    integrator_.step(0.0f);
    EXPECT_NEAR(integrator_.out(), 0.0f, 1e-9f);
}

TEST_F(IntegratorTest, SerializeDeserializeRoundTrip) {
    integrator_.step(1.0f);
    const auto snapshot = integrator_.serializeJson();
    Integrator restored;
    restored.initialize(default_config_);
    restored.deserializeJson(snapshot);
    EXPECT_EQ(integrator_.serializeJson(), restored.serializeJson());
}

} // namespace
} // namespace liteaero::control
```

### Rules — Testing

- Test names follow `MethodName_ConditionUnderTest_ExpectedBehavior` or `GivenX_WhenY_ThenZ`.
- Tests are independent; no shared mutable state between test cases.
- Use `EXPECT_NEAR` (not `EXPECT_EQ`) for floating-point comparisons; choose an appropriate tolerance.
- Mock only external dependencies (sensors, I/O); do not mock the class under test.
- Every serializable class has a round-trip test.

---

## Build System

- Use **CMake** (3.16+).
- Tests are built and run with `ctest` or `cmake --build . --target test`.
- Test targets are defined in `test/<subsystem>/CMakeLists.txt`.
- Enable sanitizers in the `Debug` configuration.

```cmake
target_compile_options(my_target PRIVATE
    $<$<CONFIG:Debug>:-fsanitize=address,undefined>
)
```

---

## External Dependency Management

### License Policy

**Prefer permissive open-source licenses.** Acceptable licenses, in order of preference:

| License | Notes |
| --- | --- |
| MIT | Preferred. Maximum compatibility. |
| BSD-2-Clause / BSD-3-Clause / Clear BSD | Preferred. Minimal restrictions. |
| Apache 2.0 | Preferred. Includes patent grant. |
| Boost Software License 1.0 | Preferred. No attribution required in binaries. |
| ISC | Preferred. Equivalent to MIT in practice. |
| LGPL v2.1 / v3 | Acceptable **only** with dynamic linking. Avoid static linking. |
| GPL (any version) | **Avoid.** Copyleft propagates to the entire linked binary. |
| Proprietary / Commercial | **Avoid** unless there is no open-source alternative and explicit approval is obtained. |

Always record the license of every dependency in `docs/dependencies/README.md` and in the
dependency registry comment in `CMakeLists.txt`.

### Conan 2.x — Primary Integration Method

C++ dependencies are managed with [Conan 2.x](https://docs.conan.io/2/). All packages
that are available in ConanCenter are declared in `conanfile.txt`. CMake consumes them
via `find_package()` after the Conan toolchain is loaded.

**Workflow:**

```bash
# Install dependencies (run once, or after changing conanfile.txt)
conan install . --output-folder=build --build=missing

# Configure — toolchain sets CMAKE_PREFIX_PATH to Conan package directories
cmake -B build -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake -DCMAKE_BUILD_TYPE=Release

# Build
cmake --build build --config Release
```

**`conanfile.txt` structure:**

```ini
[requires]
eigen/3.4.0
nlohmann_json/3.12.0
gtest/1.14.0
protobuf/3.21.12

[tool_requires]
cmake/4.1.2

[generators]
CMakeDeps
CMakeToolchain
```

**`CMakeLists.txt` consumption:**

```cmake
# All Conan packages are found via find_package — no FetchContent for these.
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(nlohmann_json REQUIRED)
find_package(GTest REQUIRED)
find_package(protobuf REQUIRED CONFIG)
```

Rules:

- All new dependencies with ConanCenter packages are added to `conanfile.txt` first.
- Pin to a specific version — never use version ranges.
- Record the library name, version, and license in `conanfile.txt` as a comment and in
  the `CMakeLists.txt` registry comment block.

### FetchContent — Fallback for Non-ConanCenter Libraries

Use CMake `FetchContent` for libraries not available in ConanCenter or where the Conan
packaging is unsuitable (e.g., header-only libraries with unusual build structures).

There are two sub-patterns depending on whether the upstream `CMakeLists.txt` is compatible:

**Pattern 1a — Upstream CMakeLists.txt is compatible:**

```cmake
include(FetchContent)
FetchContent_Declare(
    some_lib                        # MIT license
    GIT_REPOSITORY https://github.com/org/some_lib.git
    GIT_TAG        v1.2.3
    GIT_SHALLOW    TRUE
)
FetchContent_MakeAvailable(some_lib)
```

**Pattern 1b — Upstream CMakeLists.txt is incompatible or library is header-only:**

```cmake
FetchContent_Declare(
    mcap                            # MIT license — foxglove/mcap
    GIT_REPOSITORY https://github.com/foxglove/mcap.git
    GIT_TAG        releases/cpp/v1.4.0
)
FetchContent_GetProperties(mcap)
if(NOT mcap_POPULATED)
    FetchContent_Populate(mcap)
    add_library(mcap_headers INTERFACE)
    target_include_directories(mcap_headers SYSTEM INTERFACE
        ${mcap_SOURCE_DIR}/cpp/mcap/include
    )
endif()
```

Rules:

- Always pin to a specific **tag or commit SHA** — never `main` or `master`.
- Use `GIT_SHALLOW TRUE` when pinning to a tag; omit it when pinning to a bare SHA.
- Record the library name, version/SHA, and license in a comment next to the `FetchContent_Declare` call.

### Dependency Registry

Maintain this table in the root `CMakeLists.txt` as a comment header, and in `docs/dependencies/README.md`:

```text
#   Dependency       | Version         | License      | Method
#   -----------------|-----------------|--------------|----------------------
#   Eigen3           | 3.4.0           | MPL-2        | Conan (find_package)
#   nlohmann_json    | 3.12.0          | MIT          | Conan (find_package)
#   GTest            | 1.14.0          | BSD-3-Clause | Conan (find_package)
#   protobuf         | 3.21.12         | BSD-3-Clause | Conan (find_package)
#   mcap             | v1.4.0          | MIT          | FetchContent (1b)
```

---

## Error Handling

- Use exceptions for programming errors and unrecoverable state violations (`std::logic_error`, `std::runtime_error`).
- Use return codes or `std::optional`/`std::expected` for expected failure modes (e.g., waypoint not found).
- Never use exceptions for normal control flow.
- Assert preconditions at function entry in debug builds using `LAF_ASSERT` or `assert`.
- Do not `catch (...)` silently; always log or re-throw.

---

## Code Style

- Indentation: **4 spaces** (no tabs).
- Brace style: **K&R** (opening brace on same line for functions and control structures).
- Line length: **120 characters** maximum.
- Use `clang-format` with the project's `.clang-format` file for automated formatting.
- Use `clang-tidy` for static analysis.

### clang-format baseline

```yaml
# .clang-format
BasedOnStyle: Google
IndentWidth: 4
ColumnLimit: 120
AccessModifierOffset: -4
```
