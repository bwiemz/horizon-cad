# Phase 47 — Python Scripting API (Phase 1)

## Goal

Embedded Python for automation and power users: a real CPython interpreter that
executes scripts against a `horizon` API module bound with pybind11.

## Scope (Phase 1)

- Embedded interpreter (`hz::scripting`, `hz_scripting`).
- `horizon` API module bound with pybind11:
  - `horizon.Vec3` — construct, `x/y/z`, `+ - *`, `length/normalized/dot/cross`, repr.
  - Reference geometry: `DatumPlane/DatumAxis/DatumPoint` + every `refgeo`
    construction (fallible ones return `None`).
  - `horizon.Document` (the `ScriptContext` facade) — query the model
    (`feature_count`, `solid_face_count`, `solid_shell_count`, `last_error`) and
    author it (`add_rectangle_sketch`, `add_extrude`, `add_linear_pattern`,
    `add_datum_plane/axis/point`, `rebuild`).
- `ScriptEngine`:
  - One embedded interpreter per process (function-local-static
    `scoped_interpreter`, torn down after every engine is gone).
  - Persistent globals so it can back a REPL; `run(code)` and `eval(expr)`.
  - stdout captured into `Result.output`; Python exceptions surface as
    `Result.error` (never thrown across the C++ boundary).
  - A `ScriptContext*` is exposed to the script as the `doc` global.

Deferred to later scripting phases (noted, not built here): REPL console panel,
macro recording, undo integration, `horizon.ui`.

## Design notes

- **Facade over the Document.** Scripts use `ScriptContext`, a narrow stable
  surface, rather than the full internal classes — the binding layer stays small
  and is decoupled from internal churn.
- **`ScriptEngine` is pimpl'd** so the public header carries no pybind11 include.
- **Interpreter-lifetime bug avoided:** the engine's `globals` is a `py::object`
  (null default — no Python call) assigned a dict only *after* the interpreter
  is live. A `py::dict` *member* would call `PyDict_New` during member-init,
  before the interpreter exists, and crash.

## Build / CI

- New CMake option `HZ_ENABLE_SCRIPTING` (default ON). Top-level detection:
  `find_package(Python3 COMPONENTS Interpreter Development.Embed)` +
  `find_package(pybind11 CONFIG)` (both `QUIET`); the module and its tests build
  only when both are found (`HZ_SCRIPTING_ENABLED`). A dev without Python still
  builds everything else; the find is quiet so a missing dep never hard-fails
  configuration.
- Uses the **system Python** interpreter (present on both Linux and Windows CI
  runners) — no vcpkg CPython source build.
- CI installs Python via `actions/setup-python` and pybind11 via pip, exposing
  them to CMake with `pybind11_DIR` / `Python3_ROOT_DIR`. `PYBIND11_FINDPYTHON`
  is ON so pybind11 reuses the same interpreter FindPython3 located.

## Tests

`test_ScriptEngine` (10): stdout capture, syntax/runtime error reporting,
persistent globals, `eval` repr, Vec3 binding, reference-geometry binding
(incl. `None` on degenerate input), and end-to-end document authoring — box,
linear pattern, and transparent datum — driven entirely from Python.
