// =============================================================================
// Implementation: bicycle_sim (see bicycle_sim.hpp).
// Calls: sim_tune:: (near-goal gains, reverse phase-D factors, accel clamp, stop
//        tolerances inside TrackPathBicycle).
// =============================================================================

#include "autopark/bicycle_sim.hpp"
#include "autopark/sim_tune.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace autopark {

namespace {

constexpr double kPi = 3.14159265358979323846;

double Clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

double NormalizeAngle(double a) {
  while (a > kPi) a -= 2.0 * kPi;  // fold to principal range.
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

double ReverseSteerLimitScale(double dist_goal, double dist_start, double end_frac) {
  if (dist_start < 1e-6) return 1.0;  // avoid bad ratio when start == goal.
  const double r = dist_goal / dist_start;
  if (r > 0.5) return 1.0;  // full steer limit until halfway to goal along start distance.
  return end_frac + (1.0 - end_frac) * (2.0 * r);
}

double ReverseSteerNearGoalScale(double dist_goal, double d_full, double end_frac) {
  if (dist_goal >= d_full) return 1.0;  // taper only inside near-goal disk.
  return end_frac + (1.0 - end_frac) * (dist_goal / d_full);
}

double LateralErrorCenterVsGoal(double x, double y, double gx, double gy, double goal_yaw) {
  const double dx = x - gx;
  const double dy = y - gy;
  return std::abs(-dx * std::sin(goal_yaw) + dy * std::cos(goal_yaw));
}

double YawParallelErrorRad(double yaw_a, double yaw_b) {
  const double d0 = std::abs(NormalizeAngle(yaw_a - yaw_b));
  const double d1 = std::abs(NormalizeAngle(yaw_a - yaw_b - kPi));
  return std::min(d0, d1);
}

size_t FindLookaheadIndex(double px, double py, const std::vector<Point>& ref, size_t start_idx, double lookahead_dist) {
  const size_t n = ref.size();
  for (size_t i = start_idx; i < n; ++i) {  // first ref point at least lookahead away from vehicle.
    if (std::hypot(ref[i].x - px, ref[i].y - py) >= lookahead_dist) {
      return i;
    }
  }
  return n > 0 ? n - 1 : 0;  // clamp to end if path shorter than lookahead.
}

}  // namespace

double PolylineLength(const std::vector<Point>& pts) {
  if (pts.size() < 2) return 0.0;  // no segment → zero length.
  double acc = 0.0;
  for (size_t i = 1; i < pts.size(); ++i) {  // sum Euclidean edge lengths.
    acc += std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
  }
  return acc;
}

std::vector<Point> RefXYFromPolyline(const std::vector<Point>& poly, double ds) {
  if (poly.size() < 2) {
    return poly;  // cannot resample degenerate polyline.
  }
  std::vector<Point> out;
  for (size_t i = 0; i + 1 < poly.size(); ++i) {  // subdivide each segment to ~ds spacing.
    const double x0 = poly[i].x;
    const double y0 = poly[i].y;
    const double x1 = poly[i + 1].x;
    const double y1 = poly[i + 1].y;
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double seg = std::hypot(dx, dy);
    const int n = std::max(1, static_cast<int>(std::ceil(seg / ds)));
    for (int k = 0; k < n; ++k) {  // interior samples excluding segment end (deduped by next seg).
      const double t = static_cast<double>(k) / static_cast<double>(n);
      out.push_back(Point{x0 + dx * t, y0 + dy * t});
    }
  }
  out.push_back(poly.back());
  return out;
}

void PolylineTangentAt(const std::vector<Point>& poly, double dist_along, double* ox, double* oy, double* oyaw) {
  const double total = PolylineLength(poly);
  if (total < 1e-9) {  // single-point path → neutral pose.
    *ox = poly[0].x;
    *oy = poly[0].y;
    *oyaw = 0.0;
    return;
  }
  dist_along = Clamp(dist_along, 0.0, total);
  double acc = 0.0;
  for (size_t j = 0; j + 1 < poly.size(); ++j) {  // locate segment containing arc-length s.
    const double dx = poly[j + 1].x - poly[j].x;
    const double dy = poly[j + 1].y - poly[j].y;
    const double seg = std::hypot(dx, dy);
    if (acc + seg >= dist_along - 1e-9) {  // interpolate position and tangent within this edge.
      const double u = seg > 1e-9 ? (dist_along - acc) / seg : 0.0;
      *ox = poly[j].x + dx * u;
      *oy = poly[j].y + dy * u;
      *oyaw = std::atan2(dy, dx);
      return;
    }
    acc += seg;
  }
  if (poly.size() >= 2) {  // past end → use final segment direction.
    const double dx = poly.back().x - poly[poly.size() - 2].x;
    const double dy = poly.back().y - poly[poly.size() - 2].y;
    *oyaw = std::atan2(dy, dx);
  } else {
    *oyaw = 0.0;
  }
  *ox = poly.back().x;
  *oy = poly.back().y;
}

std::vector<SimPose> TrackPathBicycle(
    double sx,
    double sy,
    double syaw,
    const std::vector<Point>& ref_xy,
    bool reverse_mode,
    double wheelbase,
    double max_steer_rad,
    double max_steer_rate,
    double dt,
    double target_speed,
    double lookahead,
    double stop_tol,
    double yaw_tol,
    double goal_yaw,
    int max_steps,
    bool reverse_steer_taper,
    double reverse_steer_end_frac,
    bool reverse_phase_d,
    double reverse_phase_d_lateral_tol_m,
    double reverse_phase_d_yaw_tol_rad) {
  std::vector<SimPose> out;
  if (ref_xy.size() < 2) {
    return out;  // cannot track without a polyline reference.
  }
  const double gx = ref_xy.back().x;
  const double gy = ref_xy.back().y;
  const double dist_start = std::hypot(sx - gx, sy - gy);
  double x = sx;
  double y = sy;
  double yaw_v = syaw;
  double v = 0.0;
  double steer_prev = 0.0;
  size_t idx = 0;
  double t = 0.0;
  bool reverse_phase_d_sticky = false;

  for (int step = 0; step < max_steps; ++step) {  // integrate bicycle + Pure Pursuit until stop or cap.
    const double dist_goal = std::hypot(x - gx, y - gy);
    double ld_eff = lookahead;
    double spd_abs = std::abs(target_speed);
    if (dist_goal < sim_tune::kNearGoalShrinkDistM) {  // shrink lookahead and creep speed near goal.
      const double g = std::max(0.2, dist_goal / sim_tune::kNearGoalShrinkDistM);
      ld_eff = std::max(sim_tune::kNearGoalLdEffMin, lookahead * g);
      spd_abs = std::max(sim_tune::kNearGoalSpeedAbsMin, std::abs(target_speed) * g);
    }
    if (reverse_mode) {  // avoid overly short reverse lookahead (stability).
      ld_eff = std::max(ld_eff, sim_tune::kReverseLookaheadFloorM);
    }
    idx = FindLookaheadIndex(x, y, ref_xy, idx, ld_eff);
    const double tx = ref_xy[idx].x;
    const double ty = ref_xy[idx].y;
    double target_angle = std::atan2(ty - y, tx - x);
    double alpha = NormalizeAngle(target_angle - yaw_v);
    if (reverse_mode) {  // flip heading error when driving backwards.
      alpha = NormalizeAngle(alpha + kPi);
    }
    double steer = std::atan2(2.0 * wheelbase * std::sin(alpha), ld_eff);
    if (reverse_mode) {  // consistent steer sign with rearward motion.
      steer = -steer;
    }
    bool phase_d_active = false;
    if (reverse_mode && reverse_phase_d) {  // optional straight-in phase when aligned with slot.
      const double dyaw_align = YawParallelErrorRad(yaw_v, goal_yaw);
      const double lat_err = LateralErrorCenterVsGoal(x, y, gx, gy, goal_yaw);
      const bool in_band =
          dyaw_align <= reverse_phase_d_yaw_tol_rad && lat_err <= reverse_phase_d_lateral_tol_m;
      const bool out_band = dyaw_align > sim_tune::kReversePhaseDExitYawFactor * reverse_phase_d_yaw_tol_rad ||
          lat_err > sim_tune::kReversePhaseDExitLatFactor * reverse_phase_d_lateral_tol_m;
      if (reverse_phase_d_sticky) {
        if (out_band) {  // hysteresis exit when error grows.
          reverse_phase_d_sticky = false;
        }
      } else if (in_band) {  // enter phase-D when sufficiently aligned.
        reverse_phase_d_sticky = true;
      }
      if (reverse_phase_d_sticky) {  // drive straight (zero steer) while sliding in.
        steer = 0.0;
        phase_d_active = true;
      }
    }
    double max_eff = max_steer_rad;
    if (reverse_mode && reverse_steer_taper && !phase_d_active) {  // limit steer magnitude near reverse goal.
      const double sc_r = ReverseSteerLimitScale(dist_goal, dist_start, reverse_steer_end_frac);
      const double sc_n = ReverseSteerNearGoalScale(dist_goal, sim_tune::kReverseSteerNearGoalM, reverse_steer_end_frac);
      const double sc = std::min(sc_r, sc_n);
      max_eff = max_steer_rad * sc;
    }
    const double steer_des = Clamp(steer, -max_eff, max_eff);
    steer = steer_prev + Clamp(steer_des - steer_prev, -max_steer_rate * dt, max_steer_rate * dt);
    steer_prev = steer;

    const double spd_signed = reverse_mode ? -spd_abs : spd_abs;
    double accel = 2.0 * (spd_signed - v);
    accel = Clamp(accel, -sim_tune::kMaxAccelMs2, sim_tune::kMaxAccelMs2);
    const double v_next = v + accel * dt;
    x += v_next * std::cos(yaw_v) * dt;
    y += v_next * std::sin(yaw_v) * dt;
    yaw_v += (v_next / wheelbase) * std::tan(steer) * dt;
    yaw_v = NormalizeAngle(yaw_v);
    v = v_next;
    const double vx = v * std::cos(yaw_v);
    const double vy = v * std::sin(yaw_v);

    SimPose p;
    p.t = t;
    p.x = x;
    p.y = y;
    p.yaw = yaw_v;
    p.v = v;
    p.vx = vx;
    p.vy = vy;
    p.steer = steer;
    p.reverse_mode = reverse_mode;
    p.reverse_phase_d = phase_d_active;
    out.push_back(p);
    t += dt;

    if (reverse_mode && phase_d_active) {
      if (std::abs(x - gx) <= sim_tune::kReverseStopCenterXTolM && std::abs(y - gy) <= sim_tune::kReverseStopCenterYTolM) {
        break;
      }
    }
    if (std::hypot(x - gx, y - gy) < stop_tol && std::abs(NormalizeAngle(yaw_v - goal_yaw)) < yaw_tol) {
      break;
    }
  }
  return out;
}

}  // namespace autopark
