# autopark_cpp

C++ parking planner: reads a scene JSON, plans lane transfer + forward approach + reverse reference, optionally runs kinematic simulation, and writes result JSON and/or SVG plots.

This document is for **testers who have copied the entire `autopark_cpp` folder to their PC**. All commands assume the **current working directory is inside that folder** (the directory that contains `CMakeLists.txt`, `data\`, `src\`, and `third_party\`).

---

## Prerequisites (Windows)

1. **CMake** 3.20 or newer. Check: `cmake --version`
2. A **C++17** toolchain, for example:
   - **Visual Studio** with “Desktop development with C++”, or  
   - **MinGW-w64** with `g++` and `cmake` on `PATH`
3. Folder layout (relative to `autopark_cpp`):
   - `data\parking_scene.json` — default map
   - `third_party\nlohmann_json\single_include\` — vendored JSON header (no internet needed to build)

---

## One-time build (Windows)

Open **PowerShell** or **Command Prompt**, `cd` to your `autopark_cpp` folder, then run:

### PowerShell

```powershell
cd path\to\your\autopark_cpp
cmake -S . -B build
cmake --build build --config Release
```

If your generator is **Ninja** or **MinGW** and `--config Release` fails, use:

```powershell
cd path\to\your\autopark_cpp
cmake -S . -B build
cmake --build build
```

### Command Prompt (CMD)

```cmd
cd /d path\to\your\autopark_cpp
cmake -S . -B build
cmake --build build --config Release
```

**Expected:** `build\autopark_run.exe` exists (Visual Studio may place it under `build\Release\autopark_run.exe` — use that path in the run commands below if needed).

Verify (PowerShell):

```powershell
Test-Path .\build\autopark_run.exe
```

If `False`, try:

```powershell
Test-Path .\build\Release\autopark_run.exe
```

---

## Run tests (working directory = `autopark_cpp`)

Replace `path\to\your\autopark_cpp` once, then copy the rest as-is **after** you are inside `autopark_cpp`. Make sure you are at the right folder `autopark_cpp` for rest of the test.

### PowerShell — single run (plan + sim + default outputs)

```powershell
.\build\autopark_run.exe --map data\parking_scene.json
```

**Outputs (relative paths):**

- `outputs\parking_result_cpp.json` — result JSON (also printed to the console)
- `outputs\parking_plot_cpp.svg` — trajectory / map plot

### PowerShell — explicit output paths

```powershell
.\build\autopark_run.exe --map data\parking_scene.json --out outputs\parking_result_cpp.json --plot-out outputs\parking_plot_cpp.svg
```

### PowerShell — batch test (every free slot → one SVG + summary)

```powershell
.\build\autopark_run.exe --map data\parking_scene.json --batch-viz
```

**Outputs:**
- [batch-viz] wrote outputs/batch_viz/summary.json (27 slots, 4 failed)
- `outputs\batch_viz\summary.json` — per-slot success / messages / SVG paths
- `outputs\batch_viz\viz_r*_i*.svg` — one SVG per tested slot

**Note:** Exit code may be **1** if some slots fail; that can still be normal. Check `outputs\batch_viz\summary.json` for details.

---

## How to view results

| Output | What to do |
|--------|------------|
| `outputs\parking_result_cpp.json` | Open in any text editor or JSON viewer. |
| `outputs\parking_plot_cpp.svg` | Double-click to open in **Edge**, **Chrome**, or another browser. |
| `outputs\batch_viz\*.svg` | Same as above — open each SVG in a browser. |
| `outputs\batch_viz\summary.json` | Open in an editor; lists `success`, `message`, and `svg` path per slot. |

Optional: open the main plot after a single run (if your shell supports it):

```powershell
.\build\autopark_run.exe --map data\parking_scene.json --show-plot
```

---

## Quick pass/fail checklist

- [ ] `cmake --build` completes with no errors.
- [ ] `build\autopark_run.exe` runs without “scene file not found”.
- [ ] Single run: console shows `[ok] wrote outputs\parking_result_cpp.json` (and plot line unless `--no-plot`).
- [ ] `outputs\parking_plot_cpp.svg` opens and shows the map and polylines.
- [ ] Batch run: `outputs\batch_viz\summary.json` exists and `outputs\batch_viz\` contains multiple `viz_*.svg` files.

---

## Project layout (relative to `autopark_cpp`)

- `data\` — scene JSON and generated maps
- `third_party\nlohmann_json\` — vendored **nlohmann/json** (header-only)
- `src\`, `include\autopark\` — source code
- `build\` — CMake build tree (created locally; not shipped)
- `outputs\` — default location for JSON/SVG (created on run)

More detailed test cases: see `TEST_REPORT.md`.
