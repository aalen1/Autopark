#pragma once

// =============================================================================
// Module: geometry_lane
// -----------------------------------------------------------------------------
// Purpose:
//   Parking layout math from JSON: slot rectangles, row-dependent lane center
//   Y, travel direction, slot yaw, ego/target lane group labels, road lookup
//   by id, and small utilities (merge polylines, clamp, point-in-rect).
//
// Inputs:
//   - `json` slices: `parking` config, optional `roads` array (x,y,w,h,id).
//   - Scalar indices (row, idx) and ego position for group classification.
//
// Outputs:
//   - Rect / Point values, lane parameters, optional RoadRectById, strings
//     "upper"/"lower" for lane groups.
//
// Calls (dependencies):
//   - nlohmann::json, autopark/types.hpp.
//
// Used by:
//   - connector_transfer, forward_overshoot, parking_simulate, viz_svg, main.
// =============================================================================

#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "autopark/types.hpp"

namespace autopark {

using json = nlohmann::json;

/// Append `b` to `a`; if endpoints coincide within 5 cm, drop `b.front()`.
std::vector<Point> MergePolylines(const std::vector<Point>& a, const std::vector<Point>& b);
/// Uniform samples on segment a->b (inclusive), for straight approach/reverse stitching.
std::vector<Point> SampleStraightSegment(const Point& a, const Point& b, double step_m);

double Clamp(double v, double lo, double hi);
bool PointInRect(double x, double y, const Rect& r);

Rect SlotRect(const json& cfg, int row, int idx);
double SlotYaw(int row);
double LaneYForRow(int row);
double LaneTravelUnitX(int row);

std::optional<Rect> RoadRectById(const json& roads, const std::string& id);
std::string TargetLaneGroup(int row);
std::string EgoLaneGroup(double ex, double ey, const json& roads);

}  // namespace autopark
