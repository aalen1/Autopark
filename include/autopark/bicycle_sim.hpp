#pragma once

// =============================================================================
// Module: bicycle_sim
// -----------------------------------------------------------------------------
// Purpose:
//   Kinematic bicycle simulation along a dense XY reference: resamples polylines
//   to arc-length spacing, queries tangents, and integrates Pure Pursuit with
//   steering rate limits. Supports forward and reverse (negative speed) modes
//   and optional reverse-phase-D exit logic near the goal.
//
// Inputs:
//   - Start pose (sx, sy, syaw), reference polyline, wheelbase, steer limits
//     and rates, dt, speeds, lookahead, stop/yaw tolerances, goal yaw, caps.
//   - Helper APIs: polyline length, tangent at distance, ref resampling.
//
// Outputs:
//   - std::vector<SimPose> time series (position, velocity, steer, flags).
//
// Calls (dependencies):
//   - sim_tune.hpp for many default-related constants in call sites.
//   - types.hpp (Point, SimPose).
//
// Used by:
//   - parking_simulate (forward and reverse segments), main (PolylineLength in
//     status/JSON fields).
// =============================================================================

#include <vector>

#include "autopark/types.hpp"

namespace autopark {

std::vector<Point> RefXYFromPolyline(const std::vector<Point>& poly, double ds);
double PolylineLength(const std::vector<Point>& pts);
void PolylineTangentAt(const std::vector<Point>& poly, double dist_along, double* ox, double* oy, double* oyaw);

/// Pure Pursuit + bicycle model; `reverse_mode` uses negative speed and tail-in tracking.
std::vector<SimPose> TrackPathBicycle(
    double sx,
    double sy,
    double syaw,
    const std::vector<Point>& ref_xy,
    bool reverse_mode,
    double wheelbase,
    double max_steer_rad,
    double max_steer_rate,
    double dt,
    double target_speed,
    double lookahead,
    double stop_tol,
    double yaw_tol,
    double goal_yaw,
    int max_steps,
    bool reverse_steer_taper,
    double reverse_steer_end_frac,
    bool reverse_phase_d,
    double reverse_phase_d_lateral_tol_m,
    double reverse_phase_d_yaw_tol_rad);

}  // namespace autopark
