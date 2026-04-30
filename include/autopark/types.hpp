#pragma once

// =============================================================================
// Module: types (shared data model)
// -----------------------------------------------------------------------------
// Purpose:
//   Defines value types used across planning, kinematic simulation, and SVG
//   export. No algorithms; structs carry inputs/outputs between modules.
//
// Inputs / outputs:
//   - Consumed as fields inside PlanResult and standalone parameters.
//   - PlanResult aggregates planner polylines, geometry waypoints, sim traces,
//     and success / message flags for one target slot.
//
// Calls (dependencies):
//   - Standard library only (<string>, <vector>).
//
// Used by:
//   - geometry_lane, connector_transfer, forward_overshoot, reverse_bezier,
//     axis_fillet, bicycle_sim, parking_simulate, viz_svg, main.
// =============================================================================

#include <string>
#include <vector>

namespace autopark {

struct Point {
  double x{0.0};
  double y{0.0};
};

struct Rect {
  double x{0.0};
  double y{0.0};
  double w{0.0};
  double h{0.0};
};

struct VehiclePoseSample {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

/// One timestep from `TrackPathBicycle` (position, velocity, steer, reverse / phase-D flags).
struct SimPose {
  double t{0.0};
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
  double v{0.0};
  double vx{0.0};
  double vy{0.0};
  double steer{0.0};
  bool reverse_mode{false};
  bool reverse_phase_d{false};
};

/// Planner output and optional kinematic simulation results for one target slot.
struct PlanResult {
  int target_row{0};
  int target_idx{0};
  Rect slot{};
  Point approach_near{};
  Point staging{};
  std::vector<Point> forward_polyline;
  std::vector<Point> reverse_polyline;
  std::vector<Point> transfer_polyline;
  std::vector<Point> forward_on_target_lane;
  std::vector<SimPose> sim_forward;
  std::vector<SimPose> sim_reverse;
  bool success{false};
  bool sim_success{false};
  std::string message;
  std::string sim_message;
  /// Runtime collision source summary (map/sensor mix) for experiment debugging.
  std::string collision_model_debug;
};

}  // namespace autopark

/// Short alias: use `ap::PlanResult`, `ap::sim_tune::...` (same as fully qualifying `autopark`).
namespace ap = autopark;
