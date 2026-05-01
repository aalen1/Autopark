// =============================================================================
// Implementation: forward_overshoot (see forward_overshoot.hpp).
// Calls: connector_transfer (ChainInRoads), geometry_lane (Clamp, RoadRectById,
//        LaneTravelUnitX).
// =============================================================================

#include "autopark/forward_overshoot.hpp"

#include <cmath>

#include "autopark/connector_transfer.hpp"
#include "autopark/geometry_lane.hpp"

namespace autopark {

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
    double vehicle_length_m) {
  double near_x = 0.0;
  double stage_x = 0.0;

  // Near point and staging x from ego vs slot center so overshoot lies on the intended approach side.
  if (std::abs(ex - slot_cx) < 1e-3) {  // aligned ego uses row travel direction for left/right convention.
    const double ux = LaneTravelUnitX(row);
    near_x = slot_cx - ux * near_m;
    stage_x = slot_cx + ux * overshoot_m;
  } else if (ex < slot_cx) {  // ego left of slot → near on left, overshoot to the right.
    near_x = slot_cx - near_m;
    stage_x = slot_cx + overshoot_m;
  } else {  // ego right of slot → mirror near/overshoot.
    near_x = slot_cx + near_m;
    stage_x = slot_cx - overshoot_m;
  }
  near_x = Clamp(near_x, 0.4, map_w - 0.4);
  stage_x = Clamp(stage_x, 0.4, map_w - 0.4);

  const double half_len = 0.5 * vehicle_length_m;
  const double pad_x = 0.35;
  const auto up = RoadRectById(roads, "upper_main_lane");
  if (up.has_value() && row <= 1) {  // keep upper-lane rows inside drivable X band.
    const double x_min = up->x + pad_x + half_len + 0.25;
    const double x_max = up->x + up->w - pad_x - half_len - 0.25;
    near_x = Clamp(near_x, x_min, x_max);
    stage_x = Clamp(stage_x, x_min, x_max);
  }
  const auto lo = RoadRectById(roads, "lower_main_lane");
  if (lo.has_value() && row >= 2) {  // same for lower main lane rows.
    const double x_min = lo->x + pad_x + half_len + 0.25;
    const double x_max = lo->x + lo->w - pad_x - half_len - 0.25;
    near_x = Clamp(near_x, x_min, x_max);
    stage_x = Clamp(stage_x, x_min, x_max);
  }

  std::vector<Point> three{{ex, ey}, {near_x, lane_y}, {stage_x, lane_y}};
  if (ChainInRoads(three, roads)) {  // prefer explicit near waypoint if roads allow.
    return std::make_tuple(three, Point{near_x, lane_y}, Point{stage_x, lane_y});
  }
  std::vector<Point> two{{ex, ey}, {stage_x, lane_y}};
  if (ChainInRoads(two, roads)) {  // fallback direct leg to staging if middle point invalid.
    return std::make_tuple(two, Point{near_x, lane_y}, Point{stage_x, lane_y});
  }
  return std::nullopt;
}

}  // namespace autopark

