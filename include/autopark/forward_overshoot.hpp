#pragma once

// =============================================================================
// Module: forward_overshoot
// -----------------------------------------------------------------------------
// Purpose:
//   Builds the forward driving reference on the target row’s lane: from ego
//   toward a near point beside the slot and a staging point past the slot
//   center (overshoot), clipped to main-lane X limits when road ids exist.
//   Validates the broken line against `roads` using chained segment tests.
//
// Inputs:
//   - Ego position, slot center X, lane Y, row, map width, overshoot and
//     near distances, `roads` JSON, vehicle length (for lane padding).
//
// Outputs:
//   - Optional tuple: polyline vertices, approach_near Point, staging Point;
//     nullopt if no feasible chain inside roads.
//
// Calls (dependencies):
//   - connector_transfer: ChainInRoads.
//   - geometry_lane: Clamp, RoadRectById, LaneTravelUnitX.
//   - nlohmann::json, types.hpp.
//
// Used by:
//   - main (PlanAndBuildReference).
// =============================================================================

#include <optional>
#include <tuple>
#include <vector>

#include <nlohmann/json.hpp>

#include "autopark/types.hpp"

namespace autopark {

using json = nlohmann::json;

/// Forward polyline on target lane with near point and staging point; `roads` clips to drivable strips.
std::optional<std::tuple<std::vector<Point>, Point, Point>> BuildForwardPathOvershoot(
    double ex,
    double ey,
    double slot_cx,
    double lane_y,
    int row,
    double map_w,
    double overshoot_m,
    double near_m,
    const json& roads,
    double vehicle_length_m);

}  // namespace autopark
