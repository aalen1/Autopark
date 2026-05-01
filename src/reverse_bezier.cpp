// =============================================================================
// Implementation: reverse_bezier (see reverse_bezier.hpp).
// Calls: none (local cubic sampling only).
// =============================================================================

#include "autopark/reverse_bezier.hpp"

#include <algorithm>
#include <cmath>

namespace autopark {

std::vector<Point> CubicBezierSamples(Point p0, Point p1, Point p2, Point p3, int n) {
  std::vector<Point> out;
  out.reserve(static_cast<size_t>(std::max(2, n)));
  for (int i = 0; i < n; ++i) {  // uniform t samples along cubic Bézier.
    const double t = static_cast<double>(i) / static_cast<double>(std::max(1, n - 1));
    const double u = 1.0 - t;
    const double x = u * u * u * p0.x + 3.0 * u * u * t * p1.x + 3.0 * u * t * t * p2.x + t * t * t * p3.x;
    const double y = u * u * u * p0.y + 3.0 * u * u * t * p1.y + 3.0 * u * t * t * p2.y + t * t * t * p3.y;
    out.push_back(Point{x, y});
  }
  return out;
}

std::vector<Point> MakeReverseBezierRef(double sx, double sy, double gx, double gy, double slot_yaw, int n) {
  const double dx = gx - sx;
  const double dy = gy - sy;
  const double llen = std::hypot(dx, dy);
  if (llen < 1e-6) {  // degenerate start/end → single goal point.
    return {Point{gx, gy}};
  }
  const double px = -dy / llen;
  const double py = dx / llen;
  const double nx = std::cos(slot_yaw + 1.57079632679489661923);
  const double ny = std::sin(slot_yaw + 1.57079632679489661923);
  const Point p0{sx, sy};
  const Point p1{sx + dx * 0.28 + nx * 1.1 + px * 0.35, sy + dy * 0.28 + ny * 1.1 + py * 0.25};
  const Point p2{sx + dx * 0.72 + nx * 0.45 + px * 0.2, sy + dy * 0.72 + ny * 0.45 + py * 0.1};
  const Point p3{gx, gy};
  return CubicBezierSamples(p0, p1, p2, p3, n);
}

}  // namespace autopark
