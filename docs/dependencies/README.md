# External Dependencies

## License Policy

Prefer permissive open-source licenses. See [guidelines/general.md](../guidelines/general.md#external-dependencies-and-licensing) for the full policy.

| License | Acceptability |
| --- | --- |
| MIT, BSD-2/3-Clause, Clear BSD, Apache 2.0, Boost, ISC | ✅ Preferred |
| LGPL (any) | ⚠️ Acceptable with dynamic linking only |
| GPL (any) | ❌ Avoid |
| Proprietary | ❌ Avoid |

## Current Dependencies

### C++ Runtime Dependencies

| Library | Version | License | Integration | Purpose |
| --- | --- | --- | --- | --- |
| [Eigen3](https://eigen.tuxfamily.org) | 3.4.0 | MPL-2 | Conan (`find_package`) | Linear algebra — matrices, vectors, state-space |
| [nlohmann/json](https://github.com/nlohmann/json) | 3.12.0 | MIT | Conan (`find_package`) | JSON serialization / deserialization |
| [protobuf](https://github.com/protocolbuffers/protobuf) | 3.21.12 | BSD-3-Clause | Conan (`find_package`) | Binary serialization (proto3 wire format) |
| [mcap](https://github.com/foxglove/mcap) | v1.4.0 | MIT | FetchContent (header-only, pattern 1b) | MCAP binary log format for the logger subsystem |

### C++ Test Dependencies

| Library | Version | License | Integration | Purpose |
| --- | --- | --- | --- | --- |
| [googletest](https://github.com/google/googletest) | 1.14.0 | BSD-3-Clause | Conan (`find_package`) | Unit testing (gtest + gmock) |

### Python Dependencies

Declared in `python/pyproject.toml` when Python tooling is first added. Python version
pinned in `python/.python-version`.

## Integration Methods

### Conan 2.x (Primary)

All libraries with ConanCenter packages are managed via Conan. Conan generates
`CMakeDeps` config files and a `CMakeToolchain` file that populates `CMAKE_PREFIX_PATH`.
CMake `find_package()` locates packages through that path.

See [guidelines/cpp.md](../guidelines/cpp.md#external-dependency-management) for workflow
and `CMakeLists.txt` for the registry comment block.

### FetchContent Pattern 1b — Header-Only with Incompatible Build (mcap)

Used when the upstream library provides only headers, or has a build system incompatible
with direct `FetchContent_MakeAvailable`. Source is downloaded and an `INTERFACE` target
is defined manually:

```cmake
FetchContent_Declare(
    mcap
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
    target_compile_definitions(mcap_headers INTERFACE
        MCAP_COMPRESSION_NO_LZ4
        MCAP_COMPRESSION_NO_ZSTD
    )
endif()
```

> **Note:** `FetchContent_Populate(<name>)` is deprecated in CMake 3.30. If the project
> minimum is raised to 3.28+, migrate to `FetchContent_MakeAvailable` with a cmake
> override directory.

## Adding a New Dependency

1. Check the license against the policy above.
2. If the package is in ConanCenter: add to `conanfile.txt`, use `find_package()` in `CMakeLists.txt`.
3. If not in ConanCenter: use FetchContent (pattern 1a or 1b depending on CMake compatibility).
4. Add to the `CMakeLists.txt` registry comment block.
5. Record in this document.
