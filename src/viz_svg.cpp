// =============================================================================
// Implementation: viz_svg (see viz_svg.hpp).
// Calls: geometry_lane (slot/road drawing helpers).
// =============================================================================

#include "autopark/viz_svg.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unordered_set>

#include "autopark/geometry_lane.hpp"

namespace autopark {

namespace {

using json = nlohmann::json;

constexpr double kPi = 3.14159265358979323846;
constexpr double kVizDensifyStepM = 0.45;
constexpr int kVizBodySampleStride = 50;
constexpr int kSimGhostStride = 18;

void WrapPi(double& a) {
  // Normalize heading to (-pi, pi] for stable SVG orientation.
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
}

std::vector<Point> DensifyPolyline(const std::vector<Point>& poly, double step_m) {
  if (poly.size() < 2) {
    return poly;  // Nothing to interpolate.
  }
  std::vector<Point> out;
  // Walk each segment and insert evenly spaced samples up to step_m apart.
  for (size_t i = 0; i + 1 < poly.size(); ++i) {
    const double x0 = poly[i].x;
    const double y0 = poly[i].y;
    const double x1 = poly[i + 1].x;
    const double y1 = poly[i + 1].y;
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double dist = std::hypot(dx, dy);
    if (dist < 1e-9) {
      continue;  // Skip degenerate segment.
    }
    const int n = std::max(1, static_cast<int>(std::ceil(dist / step_m)));
    // Sample along the segment (excluding duplicate endpoint until final push_back).
    for (int k = 0; k < n; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(n);
      out.push_back(Point{x0 + dx * t, y0 + dy * t});
    }
  }
  out.push_back(poly.back());
  return out;
}

std::vector<double> YawAlongDense(const std::vector<Point>& dense, bool reverse) {
  std::vector<double> yaws;
  yaws.reserve(dense.size());
  // Tangent yaw at each dense point: forward diff, or backward diff at last vertex.
  for (size_t i = 0; i < dense.size(); ++i) {
    double dx = 1.0;
    double dy = 0.0;
    if (i + 1 < dense.size()) {
      dx = dense[i + 1].x - dense[i].x;
      dy = dense[i + 1].y - dense[i].y;
    } else if (i > 0) {
      dx = dense[i].x - dense[i - 1].x;
      dy = dense[i].y - dense[i - 1].y;
    }
    double th = std::atan2(dy, dx);
    if (reverse) {
      th += kPi;  // Point vehicle nose opposite to polyline direction (reverse gear).
    }
    WrapPi(th);
    yaws.push_back(th);
  }
  return yaws;
}

std::string EscapeXml(const std::string& s) {
  std::string o;
  o.reserve(s.size() + 8);
  // Escape characters that break SVG/XML text nodes.
  for (char c : s) {
    switch (c) {
      case '&':
        o += "&amp;";
        break;
      case '<':
        o += "&lt;";
        break;
      case '>':
        o += "&gt;";
        break;
      case '"':
        o += "&quot;";
        break;
      default:
        o += c;
    }
  }
  return o;
}

void AppendPolylineAttrs(
    std::ostringstream& os,
    const std::vector<Point>& pts,
    const char* stroke,
    double sw,
    const char* extra_attrs) {
  if (pts.size() < 2) return;  // SVG polyline needs at least two vertices.
  os << "<polyline fill=\"none\" stroke=\"" << stroke << "\" stroke-width=\"" << sw << "\"";
  if (extra_attrs) {
    os << extra_attrs;
  }
  os << " points=\"";
  // Emit space-separated "x,y" pairs for the points attribute.
  for (size_t i = 0; i < pts.size(); ++i) {
    if (i) os << ' ';
    os << pts[i].x << ',' << pts[i].y;
  }
  os << "\"/>\n";
}

void AppendPolylineFromSim(std::ostringstream& os, const std::vector<SimPose>& sim, const char* stroke, double sw) {
  if (sim.size() < 2) return;  // Need a segment to draw simulated trajectory.
  os << "<polyline fill=\"none\" stroke=\"" << stroke << "\" stroke-width=\"" << sw
     << "\" stroke-opacity=\"0.95\" points=\"";
  for (size_t i = 0; i < sim.size(); ++i) {
    if (i) os << ' ';  // Separate SVG coordinate pairs.
    os << sim[i].x << ',' << sim[i].y;
  }
  os << "\"/>\n";
}

void AppendVehicleOutline(
    std::ostringstream& os,
    double cx,
    double cy,
    double yaw,
    double L,
    double W,
    const char* stroke,
    const char* fill,
    double fill_opacity,
    double stroke_w) {
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  const double hx = 0.5 * L * c;
  const double hy = 0.5 * L * s;
  const double wx = -0.5 * W * s;
  const double wy = 0.5 * W * c;
  const double x0 = cx + hx + wx;
  const double y0 = cy + hy + wy;
  const double x1 = cx + hx - wx;
  const double y1 = cy + hy - wy;
  const double x2 = cx - hx - wx;
  const double y2 = cy - hy - wy;
  const double x3 = cx - hx + wx;
  const double y3 = cy - hy + wy;
  os << "<polygon ";
  if (fill) {
    // Filled ghost (semi-transparent) for sampled poses along a path.
    os << "fill=\"" << fill << "\" fill-opacity=\"" << fill_opacity << "\" stroke=\"" << fill
       << "\" stroke-opacity=\"0.55\" stroke-width=\"0.07\" ";
  } else {
    // Wireframe ego / final pose outline.
    os << "fill=\"none\" stroke=\"" << stroke << "\" stroke-width=\"" << stroke_w << "\" ";
  }
  os << "points=\"" << x0 << ',' << y0 << ' ' << x1 << ',' << y1 << ' ' << x2 << ',' << y2 << ' ' << x3 << ',' << y3
     << "\"/>\n";
}

double PolylineLen(const std::vector<Point>& pts) {
  double acc = 0.0;
  // Sum Euclidean segment lengths for title / stats.
  for (size_t i = 1; i < pts.size(); ++i) {
    acc += std::hypot(pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y);
  }
  return acc;
}

size_t ClosestPointIndex(const std::vector<Point>& pts, const Point& q, double* out_dist) {
  if (pts.empty()) {
    if (out_dist) *out_dist = 1e18;
    return 0;
  }
  size_t best_i = 0;
  double best_d = 1e18;
  for (size_t i = 0; i < pts.size(); ++i) {
    const double d = std::hypot(pts[i].x - q.x, pts[i].y - q.y);
    if (d < best_d) {
      best_d = d;
      best_i = i;
    }
  }
  if (out_dist) *out_dist = best_d;
  return best_i;
}

size_t ClosestSimIndex(const std::vector<SimPose>& traj, const Point& q, double* out_dist) {
  if (traj.empty()) {
    if (out_dist) *out_dist = 1e18;
    return 0;
  }
  size_t best_i = 0;
  double best_d = 1e18;
  for (size_t i = 0; i < traj.size(); ++i) {
    const double d = std::hypot(traj[i].x - q.x, traj[i].y - q.y);
    if (d < best_d) {
      best_d = d;
      best_i = i;
    }
  }
  if (out_dist) *out_dist = best_d;
  return best_i;
}

}  // namespace

std::vector<VehiclePoseSample> SampleVehicleAlongPolyline(
    const std::vector<Point>& poly,
    bool reverse,
    double densify_step_m,
    int stride) {
  std::vector<VehiclePoseSample> out;
  if (poly.size() < 2 || stride < 1) {
    return out;  // Invalid input: cannot sample.
  }
  const std::vector<Point> dense = DensifyPolyline(poly, densify_step_m);
  if (dense.empty()) {
    return out;
  }
  const std::vector<double> yaws = YawAlongDense(dense, reverse);
  // Subsample dense polyline for lightweight JSON / ghost poses.
  for (size_t i = 0; i < dense.size(); i += static_cast<size_t>(stride)) {
    out.push_back(VehiclePoseSample{dense[i].x, dense[i].y, yaws[i]});
  }
  const size_t last_i = dense.size() - 1;
  if (out.empty() || std::abs(out.back().x - dense[last_i].x) > 1e-6 || std::abs(out.back().y - dense[last_i].y) > 1e-6) {
    out.push_back(VehiclePoseSample{dense[last_i].x, dense[last_i].y, yaws[last_i]});  // Always include path endpoint.
  }
  return out;
}

bool SaveParkingPlotSvg(
    const json& scene,
    const PlanResult& res,
    double max_steer_deg,
    double overshoot_m,
    const std::string& output_path) {
  const json& m = scene.at("map");
  const double mw = scene.value("map_width", m.at("width").get<double>());
  const double mh = scene.value("map_height", m.at("height").get<double>());
  const json& cfg = scene.at("parking");
  const int sc = cfg.at("slot_count").get<int>();
  const json roads = scene.value("roads", json::array());
  const json obstacles = scene.value("obstacles", json::array());

  std::unordered_set<unsigned long long> occ;
  // Index occupied (row, index) pairs for fast lookup while drawing slots.
  for (const auto& o : scene.value("occupied_slots", json::array())) {
    const int r = o.at("row").get<int>();
    const int i = o.at("index").get<int>();
    occ.insert((static_cast<unsigned long long>(r) << 32) | static_cast<unsigned int>(i));
  }

  const auto ego = scene.at("ego_vehicle");
  const double ex = ego.at("x").get<double>();
  const double ey = ego.at("y").get<double>();
  const double e_yaw = ego.value("theta", ego.value("yaw", 0.0));
  const double L = ego.value("length", 4.8);
  const double W = ego.value("width", 2.0);

  const int tr = res.target_row;
  const int ti = res.target_idx;
  const double fwd_len = PolylineLen(res.forward_polyline);
  const double rev_len = PolylineLen(res.reverse_polyline);
  // Split reverse path at staging: [straight-back segment] + [turning segment].
  bool has_reverse_straight_segment = false;
  size_t reverse_poly_split_idx = 0;
  {
    double d_staging = 1e18;
    reverse_poly_split_idx = ClosestPointIndex(res.reverse_polyline, res.staging, &d_staging);
    has_reverse_straight_segment =
        (res.reverse_polyline.size() >= 3) &&
        (reverse_poly_split_idx > 0) &&
        (reverse_poly_split_idx + 1 < res.reverse_polyline.size()) &&
        (d_staging < 0.8);
  }

  std::ostringstream body;
  body << std::fixed << std::setprecision(3);
  body << "<g transform=\"translate(0," << mh << ") scale(1,-1)\">\n";

  body << "<rect x=\"0\" y=\"0\" width=\"" << mw << "\" height=\"" << mh
       << "\" fill=\"none\" stroke=\"black\" stroke-width=\"0.15\"/>\n";

  // Draw each road rectangle; direction flag selects lane vs generic fill color.
  for (const auto& rd : roads) {
    const double rx = rd.at("x").get<double>();
    const double ry = rd.at("y").get<double>();
    const double rw = rd.at("w").get<double>();
    const double rh = rd.at("h").get<double>();
    const int dir = rd.value("direction", 0);
    const char* fill = (dir != 0) ? "#add8e6" : "#90ee90";
    body << "<rect x=\"" << rx << "\" y=\"" << ry << "\" width=\"" << rw << "\" height=\"" << rh
         << "\" fill=\"" << fill << "\" fill-opacity=\"0.25\" stroke=\"" << fill
         << "\" stroke-width=\"0.08\"/>\n";
  }

  // Static obstacles as brown semi-opaque rectangles.
  for (const auto& ob : obstacles) {
    body << "<rect x=\"" << ob.at("x").get<double>() << "\" y=\"" << ob.at("y").get<double>()
         << "\" width=\"" << ob.at("w").get<double>() << "\" height=\"" << ob.at("h").get<double>()
         << "\" fill=\"#8b4513\" fill-opacity=\"0.55\"/>\n";
  }

  // Four parking rows × slot_count: color by occupied / target / free.
  for (int row = 0; row < 4; ++row) {
    for (int idx = 0; idx < sc; ++idx) {
      const unsigned long long key = (static_cast<unsigned long long>(row) << 32) | static_cast<unsigned int>(idx);
      const bool is_occ = occ.count(key) > 0;
      const bool is_goal = (row == tr && idx == ti);
      Rect sr = SlotRect(cfg, row, idx);
      const char* fill = is_goal ? "#98fb98" : (is_occ ? "black" : "white");
      const char* edge = is_goal ? "darkgreen" : "gray";
      const double lw = is_goal ? 0.18 : 0.08;
      body << "<rect x=\"" << sr.x << "\" y=\"" << sr.y << "\" width=\"" << sr.w << "\" height=\"" << sr.h
           << "\" fill=\"" << fill << "\" stroke=\"" << edge << "\" stroke-width=\"" << lw
           << "\" fill-opacity=\"" << (is_occ ? "0.95" : "0.85") << "\"/>\n";
    }
  }

  AppendVehicleOutline(body, ex, ey, e_yaw, L, W, "navy", nullptr, 0.0, 0.12);

  body << "<polygon fill=\"purple\" points=\""
       << (res.approach_near.x - 0.28) << ',' << res.approach_near.y << ' '
       << res.approach_near.x << ',' << (res.approach_near.y + 0.35) << ' '
       << (res.approach_near.x + 0.28) << ',' << res.approach_near.y << ' '
       << res.approach_near.x << ',' << (res.approach_near.y - 0.35) << "\"/>\n";
  body << "<circle cx=\"" << res.staging.x << "\" cy=\"" << res.staging.y
       << "\" r=\"0.38\" fill=\"darkviolet\" stroke=\"indigo\" stroke-width=\"0.06\"/>\n";

  AppendPolylineAttrs(body, res.forward_polyline, "#808080", 0.12, " stroke-opacity=\"0.65\" stroke-dasharray=\"0.45 0.35\"");
  if (has_reverse_straight_segment) {
    std::vector<Point> rev_straight_ref(
        res.reverse_polyline.begin(),
        res.reverse_polyline.begin() + static_cast<std::ptrdiff_t>(reverse_poly_split_idx + 1));
    std::vector<Point> rev_turn_ref(
        res.reverse_polyline.begin() + static_cast<std::ptrdiff_t>(reverse_poly_split_idx),
        res.reverse_polyline.end());
    // Requested visualization: straight reverse phase in blue.
    AppendPolylineAttrs(body, rev_straight_ref, "#1f77b4", 0.14, " stroke-opacity=\"0.75\" stroke-dasharray=\"0.45 0.35\"");
    AppendPolylineAttrs(body, rev_turn_ref, "#808080", 0.12, " stroke-opacity=\"0.65\" stroke-dasharray=\"0.45 0.35\"");
  } else {
    AppendPolylineAttrs(body, res.reverse_polyline, "#808080", 0.12, " stroke-opacity=\"0.65\" stroke-dasharray=\"0.45 0.35\"");
  }

  AppendPolylineFromSim(body, res.sim_forward, "#1f77b4", 0.22);
  if (has_reverse_straight_segment && !res.sim_reverse.empty()) {
    double d_sim_split = 1e18;
    const size_t sim_split_idx = ClosestSimIndex(res.sim_reverse, res.staging, &d_sim_split);
    if (sim_split_idx > 0 && sim_split_idx + 1 < res.sim_reverse.size() && d_sim_split < 1.2) {
      std::vector<SimPose> sim_rev_straight(
          res.sim_reverse.begin(),
          res.sim_reverse.begin() + static_cast<std::ptrdiff_t>(sim_split_idx + 1));
      std::vector<SimPose> sim_rev_turn(
          res.sim_reverse.begin() + static_cast<std::ptrdiff_t>(sim_split_idx),
          res.sim_reverse.end());
      AppendPolylineFromSim(body, sim_rev_straight, "#1f77b4", 0.24);
      AppendPolylineFromSim(body, sim_rev_turn, "#ff7f0e", 0.22);
    } else {
      AppendPolylineFromSim(body, res.sim_reverse, "#ff7f0e", 0.22);
    }
  } else {
    AppendPolylineFromSim(body, res.sim_reverse, "#ff7f0e", 0.22);
  }

  const auto draw_ghosts = [&](const std::vector<SimPose>& traj, const char* color, double fill_op) {
    if (traj.empty()) return;  // Nothing to draw.
    // Sparse vehicle silhouettes along simulated trajectory, plus explicit last pose.
    for (size_t i = 0; i < traj.size(); i += static_cast<size_t>(kSimGhostStride)) {
      AppendVehicleOutline(body, traj[i].x, traj[i].y, traj[i].yaw, L, W, color, color, fill_op, 0.12);
    }
    const auto& last = traj.back();
    AppendVehicleOutline(body, last.x, last.y, last.yaw, L, W, color, color, fill_op, 0.12);
  };

  if (!res.sim_forward.empty()) {
    draw_ghosts(res.sim_forward, "#1f77b4", 0.14);
  } else {
    // Fallback: ghosts from reference polyline when forward sim trace is missing.
    for (const auto& sm : SampleVehicleAlongPolyline(res.forward_polyline, false, kVizDensifyStepM, kVizBodySampleStride)) {
      AppendVehicleOutline(body, sm.x, sm.y, sm.yaw, L, W, "#1f77b4", "#1f77b4", 0.14, 0.12);
    }
  }

  if (!res.sim_reverse.empty()) {
    if (has_reverse_straight_segment) {
      double d_sim_split = 1e18;
      const size_t sim_split_idx = ClosestSimIndex(res.sim_reverse, res.staging, &d_sim_split);
      if (sim_split_idx > 0 && sim_split_idx + 1 < res.sim_reverse.size() && d_sim_split < 1.2) {
        std::vector<SimPose> sim_rev_straight(
            res.sim_reverse.begin(),
            res.sim_reverse.begin() + static_cast<std::ptrdiff_t>(sim_split_idx + 1));
        std::vector<SimPose> sim_rev_turn(
            res.sim_reverse.begin() + static_cast<std::ptrdiff_t>(sim_split_idx),
            res.sim_reverse.end());
        draw_ghosts(sim_rev_straight, "#1f77b4", 0.16);
        draw_ghosts(sim_rev_turn, "#ff7f0e", 0.16);
      } else {
        draw_ghosts(res.sim_reverse, "#ff7f0e", 0.16);
      }
    } else {
      draw_ghosts(res.sim_reverse, "#ff7f0e", 0.16);
    }
  } else {
    // Fallback: reverse-gear yaw along reference when reverse sim trace is missing.
    if (has_reverse_straight_segment) {
      std::vector<Point> rev_straight_ref(
          res.reverse_polyline.begin(),
          res.reverse_polyline.begin() + static_cast<std::ptrdiff_t>(reverse_poly_split_idx + 1));
      std::vector<Point> rev_turn_ref(
          res.reverse_polyline.begin() + static_cast<std::ptrdiff_t>(reverse_poly_split_idx),
          res.reverse_polyline.end());
      for (const auto& sm : SampleVehicleAlongPolyline(rev_straight_ref, true, kVizDensifyStepM, kVizBodySampleStride)) {
        AppendVehicleOutline(body, sm.x, sm.y, sm.yaw, L, W, "#1f77b4", "#1f77b4", 0.16, 0.12);
      }
      for (const auto& sm : SampleVehicleAlongPolyline(rev_turn_ref, true, kVizDensifyStepM, kVizBodySampleStride)) {
        AppendVehicleOutline(body, sm.x, sm.y, sm.yaw, L, W, "#ff7f0e", "#ff7f0e", 0.16, 0.12);
      }
    } else {
      for (const auto& sm : SampleVehicleAlongPolyline(res.reverse_polyline, true, kVizDensifyStepM, kVizBodySampleStride)) {
        AppendVehicleOutline(body, sm.x, sm.y, sm.yaw, L, W, "#ff7f0e", "#ff7f0e", 0.16, 0.12);
      }
    }
  }

  const double sy = SlotYaw(tr);
  // Highlight final pose: prefer last sim state, else last reference point with slot yaw.
  if (!res.sim_reverse.empty()) {
    const auto& lp = res.sim_reverse.back();
    AppendVehicleOutline(body, lp.x, lp.y, lp.yaw, L, W, "crimson", nullptr, 0.0, 0.12);
  } else if (!res.reverse_polyline.empty()) {
    const auto& lp = res.reverse_polyline.back();
    AppendVehicleOutline(body, lp.x, lp.y, sy, L, W, "crimson", nullptr, 0.0, 0.12);
  }

  body << "</g>\n";

  std::ostringstream title;
  const size_t sim_steps_total = res.sim_forward.size() + res.sim_reverse.size();
  double final_x = ex;
  double final_y = ey;
  double final_yaw = e_yaw;
  if (!res.sim_reverse.empty()) {
    const auto& lp = res.sim_reverse.back();
    final_x = lp.x;
    final_y = lp.y;
    final_yaw = lp.yaw;
  } else if (!res.reverse_polyline.empty()) {
    const auto& lp = res.reverse_polyline.back();
    final_x = lp.x;
    final_y = lp.y;
    final_yaw = sy;
  } else if (!res.sim_forward.empty()) {
    const auto& lp = res.sim_forward.back();
    final_x = lp.x;
    final_y = lp.y;
    final_yaw = lp.yaw;
  } else if (!res.forward_polyline.empty()) {
    const auto& lp = res.forward_polyline.back();
    final_x = lp.x;
    final_y = lp.y;
  }
  title << "Target slot: (" << tr << "," << ti << ")"
        << " | Sim steps total: " << sim_steps_total
        << " | Final pose: x=" << std::setprecision(2) << final_x
        << ", y=" << final_y
        << ", yaw=" << std::setprecision(3) << final_yaw << " rad"
        << " | Success: " << (res.success ? "true" : "false");
  std::string note;
  if (!res.sim_message.empty()) {
    note = res.sim_message;  // Prefer simulator status text.
  } else if (!res.message.empty()) {
    note = res.message;
  }
  if (note.size() > 72) note = note.substr(0, 69) + "...";  // Keep subtitle on one line in viewBox units.

  const std::string svg_w = "1400";
  const std::string svg_h = "900";
  const double margin_top = 3.0;
  const double vb_h = mh + margin_top;
  std::ostringstream out;
  out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
  out << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << svg_w << "\" height=\"" << svg_h
      << "\" viewBox=\"0 " << (-margin_top) << " " << mw << " " << vb_h << "\">\n";
  out << "<rect x=\"0\" y=\"" << (-margin_top) << "\" width=\"" << mw << "\" height=\"" << vb_h
      << "\" fill=\"white\"/>\n";
  out << "<text x=\"" << (0.02 * mw) << "\" y=\"" << (-0.45 * margin_top) << "\" font-size=\"1.15\" fill=\"black\">"
      << EscapeXml(title.str()) << "</text>\n";
  if (!note.empty()) {
    out << "<text x=\"" << (0.02 * mw) << "\" y=\"" << (-0.15 * margin_top) << "\" font-size=\"0.95\" fill=\"black\">"
        << EscapeXml(note) << "</text>\n";
  }
  out << body.str();
  out << "</svg>\n";

  std::ofstream f(output_path);
  if (!f) return false;
  f << out.str();
  return true;
}

}  // namespace autopark
