#pragma once

// =============================================================================
// Module: connector_transfer
// -----------------------------------------------------------------------------
// Purpose:
//   Road-network feasibility checks and construction of a piecewise polyline
//   that moves the ego between upper and lower main lanes via the shared
//   connector corridor (when JSON `roads` defines those regions).
//
// Inputs:
//   - Ego (ex, ey), ego lane group and target lane group strings, `roads` JSON.
//
// Outputs:
//   - SegmentInRoads / ChainInRoads: bool whether sampled points stay inside
//     union of road rectangles.
//   - BuildInterLaneTransfer: optional list of Points (empty if same group).
//
// Calls (dependencies):
//   - geometry_lane: PointInRect, RoadRectById, Clamp (via implementation).
//   - nlohmann::json, types.hpp.
//
// Used by:
//   - forward_overshoot (ChainInRoads), main / planner orchestration.
// =============================================================================

#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "autopark/types.hpp"

namespace autopark {

using json = nlohmann::json;

bool SegmentInRoads(const Point& a, const Point& b, const json& roads);
bool ChainInRoads(const std::vector<Point>& pts, const json& roads);

/// Piecewise path on `roads` from ego lane group to target lane group (e.g. connector between mains).
std::optional<std::vector<Point>> BuildInterLaneTransfer(
    double ex,
    double ey,
    const std::string& ego_group,
    const std::string& target_group,
    const json& roads);

}  // namespace autopark
