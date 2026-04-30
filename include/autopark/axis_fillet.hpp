#pragma once

// =============================================================================
// Module: axis_fillet
// -----------------------------------------------------------------------------
// Purpose:
//   Polyline post-processing for piecewise axis-aligned paths: remove duplicate
//   consecutive vertices, compute a nominal minimum turn radius from max steer
//   and wheelbase (via sim constants in .cpp), and replace 90° corners with
//   circular fillets so simulated steering rates are less extreme.
//
// Inputs:
//   - Polyline vertices, fillet radius R, or max_steer_deg for NominalTurnRadiusM.
//
// Outputs:
//   - New polyline (denser along arcs), or scalar radius in meters.
//
// Calls (dependencies):
//   - types.hpp only; turn radius uses a fixed wheelbase consistent with the
//     vehicle model (see NominalTurnRadiusM in .cpp).
//
// Used by:
//   - parking_simulate (forward path smoothing before / during sim).
// =============================================================================

#include <vector>

#include "autopark/types.hpp"

namespace autopark {

double NominalTurnRadiusM(double max_steer_deg);
std::vector<Point> DedupeConsecutivePolyline(const std::vector<Point>& pts);

/// Round axis-aligned 90° corners with circular arcs of radius `R`.
std::vector<Point> FilletAxisAlignedPolyline(const std::vector<Point>& vertices, double R);

}  // namespace autopark
