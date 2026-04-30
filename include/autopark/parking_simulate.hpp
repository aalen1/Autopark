#pragma once

// =============================================================================
// Module: parking_simulate
// -----------------------------------------------------------------------------
// Purpose:
//   End-to-end validation and execution of kinematic parking simulation for a
//   planned target: optionally fillet the forward polyline, verify the forward
//   path against roads / obstacles / other slots, then run bicycle tracking
//   on forward and reverse references with OBB-style collision checks against
//   roads, obstacles, and non-target slots.
//
// Inputs:
//   - Full scene `json` (map, parking, ego, roads, obstacles, occupied_slots,
//     target_slots) and mutable PlanResult from the geometric planner.
//   - Optional runtime sensor fields:
//       sensor_detections: [{x/y or cx/cy, w, h, confidence, vx, vy, ...}]
//       sensor_collision: {enabled, min_confidence, predict_horizon_s,
//                          base_inflate_m, speed_inflate_gain,
//                          replace_map_obstacles}
//   - max_steer_deg, overshoot_m (sim / geometry parameters).
//
// Outputs:
//   - Mutates PlanResult: sim_forward, sim_reverse, sim_success, sim_message;
//     ValidateForwardPolylineGeometry may clear success and set message on fail.
//
// Calls (dependencies):
//   - axis_fillet (FilletAxisAlignedPolyline, NominalTurnRadiusM, Dedupe).
//   - bicycle_sim (TrackPathBicycle, RefXYFromPolyline, PolylineLength, tangents).
//   - geometry_lane (SlotRect, PointInRect, lane/slot helpers as needed).
//   - reverse_bezier (if regenerating / validating reverse geometry).
//   - sim_tune for dt, speeds, tolerances, inflation factors.
//
// Used by:
//   - main (after planning, before JSON / SVG).
// =============================================================================

#include <nlohmann/json.hpp>

#include "autopark/types.hpp"

namespace autopark {

using json = nlohmann::json;

/// Optional fillet on forward polyline; check geometry on roads and non-target slots.
bool ValidateForwardPolylineGeometry(const json& scene, PlanResult& res, double max_steer_deg);

/// Forward/reverse `TrackPathBicycle` plus road and collision checks; fills `res.sim_*`.
void RunParkingSimulation(const json& scene, PlanResult& res, double max_steer_deg, double overshoot_m);

}  // namespace autopark
