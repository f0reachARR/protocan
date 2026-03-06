# Repository Guidelines

## Project Structure & Module Organization
This repository is a ROS 2/`colcon` workspace composed of several C++ packages:
- `protocan/`: core protocol library (`include/protocan`, `src/`, `proto/`, `tests/`).
- `protocan_device/`: device-side support library (`include/protocan_device`, `src/`, `tests/`).
- `protocan_gen/`: `protoc-gen-protocan` code generator plugin.
- `protocan_bridge/`: ROS 2 bridge node (`src/`).
- `protocan_examples/`: example proto + executable (`proto/`, `src/`).
- `tests/`: workspace-level integration tests.

Keep new code within the owning package and place public headers under each package’s `include/` tree.

## Build, Test, and Development Commands
Run from workspace root (`/home/f0reach/fortefibre_ws/src/protocan`):
- `colcon build --symlink-install`: build all packages.
- `colcon test --event-handlers console_direct+`: run package tests.
- `colcon test-result --verbose`: show test summary/failures.
- `colcon build --packages-select protocan --cmake-args -DBUILD_TESTING=ON`: build one package with tests enabled.
- `colcon test --packages-select protocan`: run tests for one package.

For package-local CMake workflows, use a dedicated build dir (for example `cmake -S protocan -B build/protocan`).

## Coding Style & Naming Conventions
- Language standard: C++17.
- Indentation: 2 spaces (follow existing files).
- Types/classes: `PascalCase` (for example `DeviceTracker`).
- Functions/variables/files: `snake_case` (for example `process_frame`, `schema_hash.cpp`).
- Headers use `#pragma once` and include paths rooted at package include dirs.

Match existing style in surrounding code; avoid large formatting-only diffs.

## Testing Guidelines
- Framework: GoogleTest (`GTest`) via CMake/`colcon`.
- Test files live in each package’s `tests/` directory and use `test_*.cpp` naming.
- Prefer scenario-based tests covering protocol behavior and edge cases (timeouts, malformed frames, partial payloads).
- Enable tests explicitly when needed with `-DBUILD_TESTING=ON`.

## Commit & Pull Request Guidelines
Recent history favors short, scoped commit subjects (for example `bridge`, `add tests`, `master`). Use concise imperative messages, optionally prefixed by package scope (for example `protocan: fix schema hash parse`).

PRs should include:
- What changed and why.
- Affected packages.
- Commands run (`colcon build`, `colcon test`) and key results.
- Linked issue/ticket when applicable.
