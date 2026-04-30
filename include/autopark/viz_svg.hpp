#pragma once

// =============================================================================
// Module: viz_svg
// -----------------------------------------------------------------------------
// Purpose:
//   Renders the parking scene and planning/simulation results as a standalone
//   SVG: map outline, roads, slots, ego, forward/reverse reference polylines,
//   simulated trajectories, and sparse vehicle pose glyphs along references.
//
// Inputs:
//   - Scene json, const PlanResult, steering/overshoot parameters for labels,
//     output file path string.
//   - SampleVehicleAlongPolyline: polyline, reverse flag, densify step, stride.
//
// Outputs:
//   - bool success writing UTF-8 SVG file; sampled poses for JSON embedding
//     elsewhere (callers in main).
//
// Calls (dependencies):
//   - geometry_lane (SlotRect, LaneYForRow, etc. for drawing).
//   - nlohmann::json, types.hpp; internal densification and SVG primitives.
//
// Used by:
//   - main (single plot and batch viz).
// =============================================================================

#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "autopark/types.hpp"

namespace autopark {

/// Write map geometry, plan polylines, and sim trajectories to a single SVG file.
bool SaveParkingPlotSvg(
    const nlohmann::json& scene,
    const PlanResult& res,
    double max_steer_deg,
    double overshoot_m,
    const std::string& output_path);

/// Densify `poly`, compute tangential yaw per step; keep every `stride` point plus last (for ghosts / JSON).
std::vector<VehiclePoseSample> SampleVehicleAlongPolyline(
    const std::vector<Point>& poly,
    bool reverse_gear,
    double densify_step_m,
    int stride);

}  // namespace autopark
