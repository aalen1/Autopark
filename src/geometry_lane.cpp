// =============================================================================
// Implementation: geometry_lane (see geometry_lane.hpp for API contract).
// Calls: none from other autopark modules; implements JSON → geometry helpers.
// =============================================================================

#include "autopark/geometry_lane.hpp"

#include <cmath>

namespace autopark {

namespace {
constexpr double kHalfPi = 1.5707963267948966;
}

double Clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

bool PointInRect(double x, double y, const Rect& r) {
  return (x >= r.x && x <= r.x + r.w && y >= r.y && y <= r.y + r.h);
}

std::vector<Point> MergePolylines(const std::vector<Point>& a, const std::vector<Point>& b) {
  if (a.empty()) return b;  // trivial merge when first chain absent.
  if (b.empty()) return a;  // keep a when second chain absent.
  std::vector<Point> out = a;
  const double d = std::hypot(a.back().x - b.front().x, a.back().y - b.front().y);
  const size_t start = (d < 0.05) ? 1 : 0;
  for (size_t i = start; i < b.size(); ++i) {  // append b, skipping duplicate joint vertex.
    out.push_back(b[i]);
  }
  return out;
}

std::vector<Point> SampleStraightSegment(const Point& a, const Point& b, double step_m) {
  std::vector<Point> out;
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double dist = std::hypot(dx, dy);
  if (dist < 1e-6) {
    return out;
  }
  const int n = std::max(1, static_cast<int>(std::ceil(dist / std::max(0.05, step_m))));
  for (int i = 0; i <= n; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(n);
    out.push_back(Point{a.x + dx * t, a.y + dy * t});
  }
  return out;
}

Rect SlotRect(const json& cfg, int row, int idx) {
  const double x = cfg.at("start_x").get<double>() + idx * cfg.at("slot_width").get<double>();
  const double w = cfg.at("slot_width").get<double>();
  const double d = cfg.at("slot_depth").get<double>();
  double y = 0.0;
  if (row == 0) {  // top row slot anchor from outer baseline.
    y = cfg.at("top_outer_baseline_y").get<double>() - d;
  } else if (row == 1) {  // upper middle row uses upper baseline.
    y = cfg.at("middle_upper_baseline_y").get<double>();
  } else if (row == 2) {  // lower middle row depth from lower baseline.
    y = cfg.at("middle_lower_baseline_y").get<double>() - d;
  } else {  // bottom row outer baseline.
    y = cfg.at("bottom_outer_baseline_y").get<double>();
  }
  return Rect{x, y, w, d};
}

double SlotYaw(int row) {
  return (row == 0 || row == 2) ? kHalfPi : -kHalfPi;
}

double LaneYForRow(int row) {
  return (row <= 1) ? 37.5 : 12.0;
}

double LaneTravelUnitX(int row) {
  return (row <= 1) ? -1.0 : 1.0;
}

std::optional<Rect> RoadRectById(const json& roads, const std::string& id) {
  for (const auto& r : roads) {  // linear search road entries by string id.
    if (r.value("id", "") == id) {  // match requested lane/connector id.
      return Rect{
          r.at("x").get<double>(),
          r.at("y").get<double>(),
          r.at("w").get<double>(),
          r.at("h").get<double>(),
      };
    }
  }
  return std::nullopt;
}

std::string TargetLaneGroup(int row) {
  return (row <= 1) ? "upper" : "lower";
}

std::string EgoLaneGroup(double ex, double ey, const json& roads) {
  const auto up = RoadRectById(roads, "upper_main_lane");
  const auto lo = RoadRectById(roads, "lower_main_lane");
  const auto cn = RoadRectById(roads, "right_connector");
  if (up.has_value() && PointInRect(ex, ey, *up)) return "upper";  // ego inside upper main lane.
  if (lo.has_value() && PointInRect(ex, ey, *lo)) return "lower";  // ego inside lower main lane.
  if (cn.has_value() && PointInRect(ex, ey, *cn)) {  // split connector by vertical half.
    return (ey < (cn->y + 0.5 * cn->h)) ? "lower" : "upper";
  }
  return (ey < 22.0) ? "lower" : "upper";  // fallback when roads JSON missing.
}

}  // namespace autopark

