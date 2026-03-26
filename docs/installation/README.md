# Installation and Build

## Prerequisites

| Tool | Version | Source |
| --- | --- | --- |
| MSYS2 ucrt64 GCC | 15.2 | `pacman -S mingw-w64-ucrt-x86_64-gcc` |
| CMake | 4.x | `pacman -S mingw-w64-ucrt-x86_64-cmake` |
| Conan | 2.x | `pacman -S mingw-w64-ucrt-x86_64-python-conan` |
| Git | Any recent | For FetchContent (mcap) |

**This project uses MSYS2 ucrt64 exclusively.** Do not use MSVC, system Python, or Conan
from any other Python environment (e.g., conda, system pip). Using the wrong Conan binary
will generate the wrong toolchain and produce a build that cannot link.

Ensure `C:/msys64/ucrt64/bin` is on your PATH before running any build commands:

```bash
export PATH="/c/msys64/ucrt64/bin:$PATH"
```

## 1. Clone the Repository

```bash
git clone https://github.com/<org>/liteaero-flight.git
cd liteaero-flight
```

## 2. Create the Conan Profile

The project requires a profile named `liteaero-gcc` that targets the MSYS2 ucrt64 GCC toolchain.
Create it at `~/.conan2/profiles/liteaero-gcc`:

```ini
[settings]
os=Windows
arch=x86_64
build_type=Release
compiler=gcc
compiler.version=14
compiler.libcxx=libstdc++11
compiler.cppstd=17

[buildenv]
CC=C:/msys64/ucrt64/bin/gcc.exe
CXX=C:/msys64/ucrt64/bin/g++.exe
```

Note: `compiler.version=14` selects pre-built ConanCenter binaries compatible with GCC 14+.
The actual installed compiler is GCC 15.2, which is ABI-compatible.

## 3. Install C++ Dependencies

Run `conan install` from the repository root. This downloads or builds all Conan-managed
dependencies and generates `build/conan_toolchain.cmake`.

```bash
PATH="/c/msys64/ucrt64/bin:$PATH" conan install . --output-folder=build --build=missing --profile=liteaero-gcc
```

For a Debug build, run a separate install pass:

```bash
PATH="/c/msys64/ucrt64/bin:$PATH" conan install . --output-folder=build-debug --build=missing --profile=liteaero-gcc --settings build_type=Debug
```

The Conan CMakeDeps generator wraps include directories and library paths in
`$<$<CONFIG:Release>:...>` (or Debug) generator expressions, so **each build type
requires its own `conan install` output folder.**

## 4. Configure and Build

```bash
# Configure
PATH="/c/msys64/ucrt64/bin:$PATH" cmake -B build -G "MinGW Makefiles" \
    -DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake \
    -DCMAKE_BUILD_TYPE=Release

# Build
PATH="/c/msys64/ucrt64/bin:$PATH" mingw32-make -C build -j$(nproc)
```

> **First configure downloads `mcap`** via CMake FetchContent. Ensure internet access is
> available or pre-populate the CMake fetch cache.

## 5. Run Tests

```bash
ctest --test-dir build --output-on-failure
```

Expected output: all tests pass except the known pre-existing failures documented in
[testing/strategy.md](../testing/strategy.md#known-failures).

## 6. VSCode Setup

Install the following extensions:

- **CMake Tools** (`ms-vscode.cmake-tools`) — configure, build, and run CTest from the IDE
- **C/C++** (`ms-vscode.cpptools`) — IntelliSense and debugging

Configure the CMake toolchain file in your workspace `settings.json`:

```json
{
    "cmake.buildDirectory": "${workspaceFolder}/build",
    "cmake.configureOnOpen": true,
    "cmake.configureSettings": {
        "CMAKE_TOOLCHAIN_FILE": "${workspaceFolder}/build/conan_toolchain.cmake"
    }
}
```

Tests appear in the **Testing** panel (beaker icon) via CTest.

## 7. Python Environment (Optional)

Python tooling uses [uv](https://docs.astral.sh/uv/). Install it if not already present:

```powershell
winget install astral-sh.uv
```

From the `python/` directory (created when Python tooling is first added):

```bash
cd python
uv sync --group dev
uv run pytest
```

## Troubleshooting

| Symptom | Likely Cause | Fix |
| --- | --- | --- |
| `Could not find Eigen3` | Conan install not run or wrong toolchain file | Run `conan install` with `--profile=liteaero-gcc` and pass `-DCMAKE_TOOLCHAIN_FILE=build/conan_toolchain.cmake` |
| `fatal error: google/protobuf/...` | Building Debug with Release-only Conan output | Run `conan install` with `--settings build_type=Debug` and use a separate output folder |
| Wrong compiler (MSVC) detected | Using wrong Conan binary (e.g., from conda) | Use `C:/msys64/ucrt64/bin/conan.exe` only |
| FetchContent download fails | No internet / proxy | Pre-populate fetch cache or use offline mirror |
| CTest shows "Build failed" | CMakeLists.txt changed, needs reconfigure | Delete `build/` and re-run `conan install` + `cmake` |
