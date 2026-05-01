// =============================================================================
// Implementation: axis_fillet (see axis_fillet.hpp).
// Calls: none from other autopark modules (internal arc geometry only).
// =============================================================================

#include "autopark/axis_fillet.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace autopark {

namespace {

constexpr double kPi = 3.14159265358979323846;

void VecUnit(double vx, double vy, double* ox, double* oy) {
  const double d = std::hypot(vx, vy);
  if (d < 1e-9) {  // avoid divide-by-zero on null segment.
    *ox = 0.0;
    *oy = 0.0;
    return;
  }
  *ox = vx / d;
  *oy = vy / d;
}

double NormPi(double a) {
  while (a <= -kPi) a += 2.0 * kPi;  // wrap angle to (-pi, pi].
  while (a > kPi) a -= 2.0 * kPi;
  return a;
}

struct FilletTriple {
  Point p_start{};
  std::vector<Point> arc;
  Point p_end{};
};

std::optional<FilletTriple> AxisAlignedFillet(Point A, Point B, Point C, double R) {
  const double ux = B.x - A.x;
  const double uy = B.y - A.y;
  const double vx = C.x - B.x;
  const double vy = C.y - B.y;
  if (std::abs(ux * vx + uy * vy) > 1e-2) {  // require perpendicular incoming/outgoing for 90° corner.
    return std::nullopt;
  }
  if ((std::abs(ux) > 1e-4 && std::abs(uy) > 1e-4) || (std::abs(vx) > 1e-4 && std::abs(vy) > 1e-4)) {
    return std::nullopt;  // reject diagonals; only axis-aligned legs.
  }
  const double Lu = std::hypot(ux, uy);
  const double Lv = std::hypot(vx, vy);
  const double R_eff = std::min(R, std::min(Lu * 0.48, Lv * 0.48));
  if (R_eff < 0.75 || Lu < R_eff + 1e-3 || Lv < R_eff + 1e-3) {  // arc must fit on both legs.
    return std::nullopt;
  }
  double u0, u1, v0, v1;
  VecUnit(ux, uy, &u0, &u1);
  VecUnit(vx, vy, &v0, &v1);
  Point p_start{B.x - R_eff * u0, B.y - R_eff * u1};
  Point p_end{B.x + R_eff * v0, B.y + R_eff * v1};
  const double cross = u0 * v1 - u1 * v0;
  if (std::abs(cross) < 1e-6) {  // need definite turn direction.
    return std::nullopt;
  }
  const bool left = cross > 0.0;
  double ox, oy;
  if (left) {  // inward normal for left turn arc center.
    ox = -u1;
    oy = u0;
  } else {  // inward normal for right turn.
    ox = u1;
    oy = -u0;
  }
  const double Ox = p_start.x + R_eff * ox;
  const double Oy = p_start.y + R_eff * oy;
  double th0 = std::atan2(p_start.y - Oy, p_start.x - Ox);
  double th1 = std::atan2(p_end.y - Oy, p_end.x - Ox);
  double delta = NormPi(th1 - th0);
  if (left) {
    if (delta < 0) delta += 2.0 * kPi;  // positive sweep for CCW arc.
  } else {
    if (delta > 0) delta -= 2.0 * kPi;  // negative sweep for CW arc.
  }
  const double arc_len = std::abs(delta) * R_eff;
  const int n_arc = std::clamp(static_cast<int>(arc_len / 0.18), 10, 48);
  std::vector<Point> arc;
  arc.reserve(static_cast<size_t>(n_arc) + 1);
  for (int k = 0; k <= n_arc; ++k) {  // discretize circular arc for polyline output.
    const double t = static_cast<double>(k) / static_cast<double>(std::max(1, n_arc));
    const double th = th0 + t * delta;
    arc.push_back(Point{Ox + R_eff * std::cos(th), Oy + R_eff * std::sin(th)});
  }
  return FilletTriple{p_start, std::move(arc), p_end};
}

}  // namespace

double NominalTurnRadiusM(double max_steer_deg) {
  const double a = std::max(0.06, max_steer_deg * kPi / 180.0);
  return std::max(2.6, 2.7 / std::tan(a));
}

std::vector<Point> DedupeConsecutivePolyline(const std::vector<Point>& pts) {
  if (pts.empty()) return {};  // nothing to clean.
  std::vector<Point> out;
  out.push_back(pts[0]);
  for (size_t i = 1; i < pts.size(); ++i) {  // drop consecutive duplicates / micro-segments.
    if (std::hypot(pts[i].x - out.back().x, pts[i].y - out.back().y) > 1e-4) {
      out.push_back(pts[i]);
    }
  }
  return out;
}

std::vector<Point> FilletAxisAlignedPolyline(const std::vector<Point>& vertices, double R) {
  if (R <= 0.0 || vertices.size() < 3) {  // need corner triple; else just dedupe.
    return DedupeConsecutivePolyline(vertices);
  }
  std::vector<Point> out;
  out.push_back(vertices[0]);
  std::vector<Point> rest(vertices.begin() + 1, vertices.end());
  while (rest.size() >= 2) {  // consume vertices, inserting fillets at eligible corners.
    const Point A = out.back();
    const Point B = rest[0];
    const Point C = rest[1];
    auto fil = AxisAlignedFillet(A, B, C, R);
    if (!fil.has_value()) {  // sharp corner not filletable → keep B and advance.
      out.push_back(B);
      rest.erase(rest.begin());
      continue;
    }
    const Point& p_start = fil->p_start;
    const auto& arc_pts = fil->arc;
    const Point& p_end = fil->p_end;
    if (std::hypot(p_start.x - A.x, p_start.y - A.y) > 1e-3) {  // add tangent leave point if not duplicate.
      out.push_back(p_start);
    }
    for (size_t j = 1; j < arc_pts.size(); ++j) {  // append arc samples (skip duplicates).
      const Point& q = arc_pts[j];
      if (out.empty() || std::hypot(q.x - out.back().x, q.y - out.back().y) > 1e-4) {
        out.push_back(q);
      }
    }
    if (std::hypot(p_end.x - out.back().x, p_end.y - out.back().y) > 1e-4) {  // close arc at tangent entry to next leg.
      out.push_back(p_end);
    }
    rest.erase(rest.begin(), rest.begin() + 2);
    rest.insert(rest.begin(), C);
  }
  if (rest.size() == 1) {  // append trailing vertex after last fillet window.
    const Point& p = rest[0];
    if (std::hypot(p.x - out.back().x, p.y - out.back().y) > 1e-4) {
      out.push_back(p);
    }
  }
  return DedupeConsecutivePolyline(out);
}

}  // namespace autopark
