#pragma once

// =============================================================================
// Module: reverse_bezier
// -----------------------------------------------------------------------------
// Purpose:
//   Generates a smooth cubic Bezier curve in world XY as the reverse-parking
//   reference: starts at staging, ends near the slot center, with control
//   points aligned to slot yaw for a plausible back-in approach.
//
// Inputs:
//   - Staging (sx, sy), goal (gx, gy), slot_yaw (rad), sample count n.
//   - CubicBezierSamples: explicit control points p0..p3 and n.
//
// Outputs:
//   - std::vector<Point> sampled along the curve (uniform t).
//
// Calls (dependencies):
//   - types.hpp only (self-contained math).
//
// Used by:
//   - main (planning), parking_simulate (optional regen / tangents), viz_svg
//     via PlanResult polylines.
// =============================================================================

#include <vector>

#include "autopark/types.hpp"

namespace autopark {

std::vector<Point> CubicBezierSamples(Point p0, Point p1, Point p2, Point p3, int n);

/// Reverse parking reference as a cubic Bezier from staging toward slot center.
std::vector<Point> MakeReverseBezierRef(double sx, double sy, double gx, double gy, double slot_yaw, int n = 48);

}  // namespace autopark
