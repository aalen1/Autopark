# Autopark (C++)

## Project Overview

This project simulates an autonomous parking system. When a user enters a parking lot, they only need to choose a target parking slot, and the system can complete the full parking process in one run.  
It supports both custom scenario testing and batch testing on a standard map, aiming to provide a fast, practical, convenient, and accurate auto-parking workflow.

### Algorithm Overview

- **Vehicle motion simulation:** The planner uses a bicycle-model-based kinematic simulation to track generated reference paths and produce physically plausible vehicle trajectories over time.
- **Obstacle avoidance strategy:** The system performs geometry and simulation-time collision checks against occupied slots, static obstacles, and optional sensor obstacles, with configurable safety inflation.
- **Path planning logic:** It builds a structured plan in stages: lane-group detection, optional inter-lane transfer through the connector, forward approach with overshoot staging, and reverse insertion toward the target slot.
- **Segmented reverse parking method:** For complex edge cases, reverse parking is handled in segments (for example, straight reverse alignment followed by turning reverse), improving robustness and final pose quality.

---

## Prerequisites (Windows)

- MinGW-w64 (or another C++17 compiler) with `g++` on `PATH`
- PowerShell or CMD
- Vendored JSON header present at:
  - `third party\nlohmann\json.hpp`

---

## Build (one-time)

Run from the `Autopark` folder:

```powershell
cd "C:\Users\aalen\Desktop\556\autopark\autopark_cpp\Autopark"
mkdir build -Force > $null
g++ -std=c++17 -O2 -Iinclude -I"third party" src/main.cpp src/geometry_lane.cpp src/connector_transfer.cpp src/forward_overshoot.cpp src/reverse_bezier.cpp src/viz_svg.cpp src/axis_fillet.cpp src/bicycle_sim.cpp src/parking_simulate.cpp -o build/autopark_run.exe
```

Verify:

```powershell
Test-Path .\build\autopark_run.exe
```

---

## TUI Usage (Terminal Command + Input Method)

Start interactive TUI:

```powershell
cd "C:\Users\aalen\Desktop\556\autopark\autopark_cpp\Autopark"
.\build\autopark_run.exe --tui --custom-map-out outputs\custom_map_tui.json
```

### TUI Menu

- `1` Set row count (`1-4`)
- `2` Set slots per row (`1-20`)
- `3` Set target slot (`row`, `index`)
- `4` Show current settings (includes geometry summary)
- `5` Run planning and export files
- `0` Exit

### Example Input Sequence

Use this sequence in the terminal after TUI starts:

```text
1
4
2
20
3
0
19
4
5
```

### TUI Outputs

- `outputs\custom_map_tui.json`
- `outputs\custom_result_r{row}_i{index}.json`
- `outputs\custom_plot_r{row}_i{index}.svg`

---

## Original Map Full Test

Run the complete batch test on the original map:

```powershell
cd "C:\Users\aalen\Desktop\556\autopark\autopark_cpp\Autopark"
.\build\autopark_run.exe --map data\parking_scene.json --batch-viz
```

Outputs:

- `outputs\batch_viz\summary.json`
- `outputs\batch_viz\viz_r*_i*.svg`

Note: exit code can be `1` when some slots fail; use `summary.json` as the final report.

---

## Result Viewing

- Open JSON files with any text editor.
- Open SVG files with Edge/Chrome/Firefox.

For detailed scenario logs, see `TEST_REPORT.md`.
