// =============================================================================
// Implementation: connector_transfer (see connector_transfer.hpp).
// Calls: geometry_lane (PointInRect, RoadRectById, Clamp).
// =============================================================================

#include "autopark/connector_transfer.hpp"

#include <cmath>

#include "autopark/geometry_lane.hpp"

namespace autopark {

bool SegmentInRoads(const Point& a, const Point& b, const json& roads) {
  const int samples = std::max(3, static_cast<int>(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / 0.5)));
  for (int i = 0; i <= samples; ++i) {  // sample segment so every point lies in some road rect.
    const double t = static_cast<double>(i) / static_cast<double>(samples);
    const double x = a.x + (b.x - a.x) * t;
    const double y = a.y + (b.y - a.y) * t;
    bool ok = false;
    for (const auto& rr : roads) {  // test sample against each road rectangle.
      const Rect r{
          rr.at("x").get<double>(),
          rr.at("y").get<double>(),
          rr.at("w").get<double>(),
          rr.at("h").get<double>(),
      };
      if (PointInRect(x, y, r)) {  // accept if inside at least one drivable region.
        ok = true;
        break;
      }
    }
    if (!ok) return false;  // reject segment if any sample leaves all roads.
  }
  return true;
}

bool ChainInRoads(const std::vector<Point>& pts, const json& roads) {
  if (pts.size() < 2) return false;  // need at least one segment to validate.
  for (size_t i = 1; i < pts.size(); ++i) {  // check each consecutive edge in the chain.
    if (!SegmentInRoads(pts[i - 1], pts[i], roads)) return false;
  }
  return true;
}

std::optional<std::vector<Point>> BuildInterLaneTransfer(
    double ex,
    double ey,
    const std::string& ego_group,
    const std::string& target_group,
    const json& roads) {
  if (ego_group == target_group) return std::vector<Point>{};  // no connector path when already on target main.
  const auto lo = RoadRectById(roads, "lower_main_lane");
  const auto up = RoadRectById(roads, "upper_main_lane");
  const auto cn = RoadRectById(roads, "right_connector");
  if (!lo.has_value() || !up.has_value() || !cn.has_value()) return std::nullopt;  // require full road set.

  const double jx = Clamp(cn->x + 0.33 * cn->w, cn->x + 0.45, cn->x + cn->w - 0.55);
  const double y_lo = Clamp(lo->y + 0.52 * lo->h, lo->y + 0.55, lo->y + lo->h - 0.5 * 4.8 - 0.45);
  const double y_up = 37.5;

  if (ego_group == "lower" && target_group == "upper") {  // climb via connector junction to upper lane Y.
    std::vector<Point> cand{{ex, ey}, {ex, y_lo}, {jx, y_lo}, {jx, y_up}};
    if (ChainInRoads(cand, roads)) return cand;  // return first feasible polyline.
  }
  if (ego_group == "upper" && target_group == "lower") {  // descend symmetrically to lower lane.
    std::vector<Point> cand{{ex, ey}, {ex, y_up}, {jx, y_up}, {jx, y_lo}};
    if (ChainInRoads(cand, roads)) return cand;
  }
  return std::nullopt;
}

}  // namespace autopark

