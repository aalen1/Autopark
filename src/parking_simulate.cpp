// =============================================================================
// Implementation: parking_simulate (see parking_simulate.hpp).
// Calls: axis_fillet, bicycle_sim, geometry_lane, reverse_bezier, sim_tune.
// =============================================================================

#include "autopark/parking_simulate.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <sstream>
#include <string>
#include <vector>

#include "autopark/axis_fillet.hpp"
#include "autopark/bicycle_sim.hpp"
#include "autopark/geometry_lane.hpp"
#include "autopark/reverse_bezier.hpp"
#include "autopark/sim_tune.hpp"

namespace autopark {

namespace detail {

using namespace sim_tune;

Rect InflateRect(Rect r, double pad) {
  return Rect{r.x - pad, r.y - pad, r.w + 2.0 * pad, r.h + 2.0 * pad};
}

std::vector<Rect> RoadsFromJson(const json& scene) {
  std::vector<Rect> out;
  for (const auto& rd : scene.value("roads", json::array())) {
    out.push_back(Rect{rd.at("x").get<double>(), rd.at("y").get<double>(), rd.at("w").get<double>(), rd.at("h").get<double>()});
  }
  return out;
}

std::vector<Rect> BuildForbiddenSlots(const json& cfg, int target_row, int target_idx) {
  const int sc = cfg.at("slot_count").get<int>();
  std::vector<Rect> out;
  for (int row = 0; row < 4; ++row) {
    for (int idx = 0; idx < sc; ++idx) {
      if (row == target_row && idx == target_idx) {
        continue;
      }
      out.push_back(SlotRect(cfg, row, idx));
    }
  }
  return out;
}

std::vector<Rect> BuildOccupiedSlotRects(const json& cfg, const json& occupied) {
  std::vector<Rect> out;
  for (const auto& o : occupied) {
    out.push_back(SlotRect(cfg, o.at("row").get<int>(), o.at("index").get<int>()));
  }
  return out;
}

std::vector<Rect> ObstaclesFromJson(const json& scene) {
  std::vector<Rect> out;
  for (const auto& o : scene.value("obstacles", json::array())) {
    out.push_back(Rect{o.at("x").get<double>(), o.at("y").get<double>(), o.at("w").get<double>(), o.at("h").get<double>()});
  }
  return out;
}

struct RuntimeObstacleModel {
  std::vector<Rect> forward_forbidden;
  std::vector<Rect> reverse_forbidden;
  bool sensor_enabled{false};
  bool replace_map_obstacles{false};
  int sensor_raw_count{0};
  int sensor_kept_count{0};
};

bool IsUpperLastSlotAfterTurn(const json& cfg, const PlanResult& res) {
  const int sc = cfg.at("slot_count").get<int>();
  return (res.target_row != 2 && res.target_row != 3) &&
      (res.target_idx == sc - 1) &&
      !res.transfer_polyline.empty();
}

void RemoveRectIfMatches(std::vector<Rect>& rects, const Rect& target) {
  std::vector<Rect> filtered;
  filtered.reserve(rects.size());
  for (const Rect& r : rects) {
    const bool same = std::abs(r.x - target.x) < 1e-6 &&
        std::abs(r.y - target.y) < 1e-6 &&
        std::abs(r.w - target.w) < 1e-6 &&
        std::abs(r.h - target.h) < 1e-6;
    if (!same) {
      filtered.push_back(r);
    }
  }
  rects = std::move(filtered);
}

std::vector<Rect> SensorDetectionsToRects(const json& scene, RuntimeObstacleModel& model) {
  const json sensor_cfg = scene.value("sensor_collision", json::object());
  const bool has_sensor_dets = scene.contains("sensor_detections") &&
      scene.at("sensor_detections").is_array() &&
      !scene.at("sensor_detections").empty();
  const bool has_frame_dets = scene.contains("sensor_frame") &&
      scene.at("sensor_frame").is_object() &&
      scene.at("sensor_frame").contains("detections") &&
      scene.at("sensor_frame").at("detections").is_array() &&
      !scene.at("sensor_frame").at("detections").empty();
  model.sensor_enabled = sensor_cfg.value("enabled", false) || has_sensor_dets || has_frame_dets;
  model.replace_map_obstacles = sensor_cfg.value("replace_map_obstacles", false);
  if (!model.sensor_enabled) {
    return {};
  }

  const json* dets = nullptr;
  if (has_sensor_dets) {
    dets = &scene.at("sensor_detections");
  } else if (has_frame_dets) {
    dets = &scene.at("sensor_frame").at("detections");
  }
  if (dets == nullptr) {
    return {};
  }

  const double min_conf = sensor_cfg.value("min_confidence", 0.55);
  const double predict_horizon_s = std::max(0.0, sensor_cfg.value("predict_horizon_s", 0.8));
  const double base_inflate_m = std::max(0.0, sensor_cfg.value("base_inflate_m", 0.25));
  const double speed_gain = std::max(0.0, sensor_cfg.value("speed_inflate_gain", 0.35));
  double map_w = scene.value("map_width", 75.0);
  double map_h = scene.value("map_height", 50.0);
  if (scene.contains("map") && scene.at("map").is_object()) {
    map_w = scene.at("map").value("width", map_w);
    map_h = scene.at("map").value("height", map_h);
  }

  std::vector<Rect> out;
  out.reserve(dets->size());
  model.sensor_raw_count = static_cast<int>(dets->size());

  for (const auto& d : *dets) {
    if (!d.is_object()) {
      continue;
    }
    if (!d.contains("w") || !d.contains("h")) {
      continue;
    }
    const double w = d.at("w").get<double>();
    const double h = d.at("h").get<double>();
    if (w <= 1e-3 || h <= 1e-3) {
      continue;
    }
    const double conf = d.value("confidence", 1.0);
    if (conf < min_conf) {
      continue;
    }

    double cx = 0.0;
    double cy = 0.0;
    if (d.contains("cx") && d.contains("cy")) {
      cx = d.at("cx").get<double>();
      cy = d.at("cy").get<double>();
    } else if (d.contains("x") && d.contains("y")) {
      const double x = d.at("x").get<double>();
      const double y = d.at("y").get<double>();
      const bool xy_is_center = d.value("xy_is_center", false);
      cx = xy_is_center ? x : (x + 0.5 * w);
      cy = xy_is_center ? y : (y + 0.5 * h);
    } else {
      continue;
    }

    const double vx = d.value("vx", 0.0);
    const double vy = d.value("vy", 0.0);
    const double speed = std::hypot(vx, vy);
    cx += vx * predict_horizon_s;
    cy += vy * predict_horizon_s;

    Rect r{cx - 0.5 * w, cy - 0.5 * h, w, h};
    r = InflateRect(r, base_inflate_m + speed_gain * speed);

    if (r.x > map_w || r.y > map_h || (r.x + r.w) < 0.0 || (r.y + r.h) < 0.0) {
      continue;
    }
    out.push_back(r);
  }

  model.sensor_kept_count = static_cast<int>(out.size());
  return out;
}

RuntimeObstacleModel BuildRuntimeObstacleModel(const json& scene, const PlanResult& res, bool upper_last_slot_after_turn) {
  const json& cfg = scene.at("parking");
  RuntimeObstacleModel model;

  const auto forbidden_slots = BuildForbiddenSlots(cfg, res.target_row, res.target_idx);
  const auto occupied_slots = BuildOccupiedSlotRects(cfg, scene.value("occupied_slots", json::array()));
  const auto static_obs = ObstaclesFromJson(scene);
  const auto sensor_obs = SensorDetectionsToRects(scene, model);

  model.forward_forbidden = forbidden_slots;
  model.forward_forbidden.insert(model.forward_forbidden.end(), static_obs.begin(), static_obs.end());
  model.forward_forbidden.insert(model.forward_forbidden.end(), sensor_obs.begin(), sensor_obs.end());

  if (model.sensor_enabled && model.replace_map_obstacles) {
    model.reverse_forbidden = sensor_obs;
  } else {
    model.reverse_forbidden = occupied_slots;
    model.reverse_forbidden.insert(model.reverse_forbidden.end(), static_obs.begin(), static_obs.end());
    model.reverse_forbidden.insert(model.reverse_forbidden.end(), sensor_obs.begin(), sensor_obs.end());
  }

  if (upper_last_slot_after_turn && res.target_idx > 0) {
    const Rect adj_left = SlotRect(cfg, res.target_row, res.target_idx - 1);
    RemoveRectIfMatches(model.reverse_forbidden, adj_left);
  }
  return model;
}

std::string CollisionModelDebugString(const RuntimeObstacleModel& model) {
  std::ostringstream oss;
  oss << "forward_forbidden=" << model.forward_forbidden.size()
      << ", reverse_forbidden=" << model.reverse_forbidden.size();
  if (model.sensor_enabled) {
    oss << ", sensor(raw=" << model.sensor_raw_count
        << ", kept=" << model.sensor_kept_count
        << ", replace_map=" << (model.replace_map_obstacles ? "true" : "false") << ")";
  } else {
    oss << ", sensor=off";
  }
  return oss.str();
}

void VehicleCorners(double cx, double cy, double yaw, double L, double W, double c[4][2]) {
  const double cth = std::cos(yaw);
  const double sth = std::sin(yaw);
  const double hx = 0.5 * L * cth;
  const double hy = 0.5 * L * sth;
  const double wx = -0.5 * W * sth;
  const double wy = 0.5 * W * cth;
  c[0][0] = cx + hx + wx;
  c[0][1] = cy + hy + wy;
  c[1][0] = cx + hx - wx;
  c[1][1] = cy + hy - wy;
  c[2][0] = cx - hx - wx;
  c[2][1] = cy - hy - wy;
  c[3][0] = cx - hx + wx;
  c[3][1] = cy - hy + wy;
}

bool CornerInRect(double px, double py, const Rect& r) {
  return PointInRect(px, py, r);
}

bool ObbHitsRect(const double corners[4][2], const Rect& r) {
  for (int i = 0; i < 4; ++i) {
    if (CornerInRect(corners[i][0], corners[i][1], r)) {
      return true;
    }
  }
  return false;
}

bool PointOnRoadsInflated(double px, double py, const std::vector<Rect>& roads, double inflate_m) {
  for (const Rect& rd : roads) {
    if (inflate_m > 0.0) {
      if (PointInRect(px, py, InflateRect(rd, inflate_m))) {
        return true;
      }
    } else {
      if (PointInRect(px, py, rd)) {
        return true;
      }
    }
  }
  return false;
}

bool ObFullyOnRoadsInflated(const double corners[4][2], const std::vector<Rect>& roads, double inflate_m) {
  for (int i = 0; i < 4; ++i) {
    if (!PointOnRoadsInflated(corners[i][0], corners[i][1], roads, inflate_m)) {
      return false;
    }
  }
  return true;
}

bool SegmentInRoads(double ax, double ay, double bx, double by, const std::vector<Rect>& roads, int samples) {
  for (int i = 0; i <= samples; ++i) {
    const double t = static_cast<double>(i) / static_cast<double>(samples);
    const double px = ax + (bx - ax) * t;
    const double py = ay + (by - ay) * t;
    if (!PointOnRoadsInflated(px, py, roads, 0.0)) {
      return false;
    }
  }
  return true;
}

bool PolylineChainInRoads(const std::vector<Point>& pts, const std::vector<Rect>& roads) {
  if (pts.size() < 2) {
    return true;
  }
  for (size_t i = 0; i + 1 < pts.size(); ++i) {
    const double ax = pts[i].x;
    const double ay = pts[i].y;
    const double bx = pts[i + 1].x;
    const double by = pts[i + 1].y;
    const double slen = std::hypot(bx - ax, by - ay);
    const int samples = static_cast<int>(std::clamp(static_cast<int>(slen / 0.22), 14, 72));
    if (!SegmentInRoads(ax, ay, bx, by, roads, samples)) {
      return false;
    }
  }
  return true;
}

bool CheckForwardCollisionGeom(
    const std::vector<Point>& poly,
    double length,
    double width,
    const std::vector<Rect>& forbidden,
    const std::vector<Rect>& roads) {
  const double total = PolylineLength(poly);
  if (total < 1e-6) {
    return true;
  }
  const int samples = std::max(30, static_cast<int>(total / 0.3));
  for (int i = 0; i <= samples; ++i) {
    const double dist = (static_cast<double>(i) / static_cast<double>(samples)) * total;
    double cx, cy, yaw;
    PolylineTangentAt(poly, dist, &cx, &cy, &yaw);
    double c[4][2];
    VehicleCorners(cx, cy, yaw, length, width, c);
    if (!ObFullyOnRoadsInflated(c, roads, 0.0)) {
      return false;
    }
    for (const Rect& fr : forbidden) {
      if (ObbHitsRect(c, fr)) {
        return false;
      }
    }
  }
  return true;
}

bool CheckForwardSim(
    const std::vector<SimPose>& traj,
    double length,
    double width,
    const std::vector<Rect>& forbidden,
    const std::vector<Rect>& roads,
    double roads_inflate_m) {
  double c[4][2];
  for (const auto& p : traj) {
    VehicleCorners(p.x, p.y, p.yaw, length, width, c);
    if (!ObFullyOnRoadsInflated(c, roads, roads_inflate_m)) {
      return false;
    }
    for (const Rect& fr : forbidden) {
      if (ObbHitsRect(c, fr)) {
        return false;
      }
    }
  }
  return true;
}

bool CheckReverseSimLenient(
    const std::vector<SimPose>& traj,
    double length,
    double width,
    const std::vector<Rect>& occupied_plus_obs,
    double obb_scale) {
  const double lc = length * obb_scale;
  const double wc = width * obb_scale;
  double c[4][2];
  for (const auto& p : traj) {
    VehicleCorners(p.x, p.y, p.yaw, lc, wc, c);
    for (const Rect& fr : occupied_plus_obs) {
      if (ObbHitsRect(c, fr)) {
        return false;
      }
    }
  }
  return true;
}

bool FinalInSlotLoose(double cx, double cy, double yaw, const Rect& slot, double slot_yaw_expected, double length, double width) {
  (void)slot_yaw_expected;
  double c[4][2];
  VehicleCorners(cx, cy, yaw, length, width, c);
  const Rect slot_pad = InflateRect(slot, kFinalSlotGeomPadM);
  int n_in = 0;
  for (int i = 0; i < 4; ++i) {
    if (PointInRect(c[i][0], c[i][1], slot_pad)) {
      ++n_in;
    }
  }
  const bool geom_ok = PointInRect(cx, cy, slot_pad) || n_in >= kFinalSlotMinCornersInPadded;
  return geom_ok;
}

bool ValidateForwardPolylineGeometryBody(const json& scene, PlanResult& res, double max_steer_deg) {
  const json& cfg = scene.at("parking");
  const bool upper_last_slot_after_turn = IsUpperLastSlotAfterTurn(cfg, res);
  const auto runtime_obs = BuildRuntimeObstacleModel(scene, res, upper_last_slot_after_turn);
  const auto roads_vec = RoadsFromJson(scene);
  const auto ego = scene.at("ego_vehicle");
  const double L = ego.value("length", 4.8);
  const double W = ego.value("width", 2.0);
  const double R = NominalTurnRadiusM(max_steer_deg);
  auto rounded = FilletAxisAlignedPolyline(res.forward_polyline, R);
  if (rounded.size() >= 2 && PolylineChainInRoads(rounded, roads_vec) &&
      CheckForwardCollisionGeom(rounded, L, W, runtime_obs.forward_forbidden, roads_vec)) {
    res.forward_polyline = std::move(rounded);
  }
  res.collision_model_debug = CollisionModelDebugString(runtime_obs);
  return CheckForwardCollisionGeom(res.forward_polyline, L, W, runtime_obs.forward_forbidden, roads_vec);
}

void RunParkingSimulationBody(const json& scene, PlanResult& res, double max_steer_deg, double overshoot_m) {
  (void)overshoot_m;
  res.sim_forward.clear();
  res.sim_reverse.clear();
  res.sim_success = false;
  res.sim_message.clear();

  if (!res.success) {
    res.sim_message = "planning failed";
    return;
  }

  const json& cfg = scene.at("parking");
  const bool upper_last_slot_after_turn = IsUpperLastSlotAfterTurn(cfg, res);
  const auto roads_vec = RoadsFromJson(scene);
  const auto runtime_obs = BuildRuntimeObstacleModel(scene, res, upper_last_slot_after_turn);
  const auto& forbidden = runtime_obs.forward_forbidden;
  const auto& occ_and_obs = runtime_obs.reverse_forbidden;
  res.collision_model_debug = CollisionModelDebugString(runtime_obs);

  const auto& ego = scene.at("ego_vehicle");
  const double ex = ego.at("x").get<double>();
  const double ey = ego.at("y").get<double>();
  const double yaw0 = ego.value("theta", ego.value("yaw", 0.0));
  const double length = ego.value("length", 4.8);
  const double width = ego.value("width", 2.0);

  const double max_steer_rad = max_steer_deg * 3.14159265358979323846 / 180.0;
  const double dt_f = kDtSim;

  auto ref_fwd_full = RefXYFromPolyline(res.forward_polyline, kRefDsForwardFull);
  const double tl_full = PolylineLength(res.forward_polyline);
  double gx_f, gy_f, goal_yaw_fwd;
  PolylineTangentAt(res.forward_polyline, std::max(0.0, tl_full - kPolylineTangentEndOffsetM), &gx_f, &gy_f, &goal_yaw_fwd);

  std::vector<SimPose> sim_f = TrackPathBicycle(
      ex,
      ey,
      yaw0,
      ref_fwd_full,
      false,
      kWheelbaseM,
      max_steer_rad,
      kMaxSteerRateRadS,
      dt_f,
      kFwdSpeed,
      kFwdLookahead,
      kFwdStopTol,
      kFwdYawTol,
      goal_yaw_fwd,
      kFwdMaxSteps,
      false,
      kReverseSteerTaperEndFrac,
      false,
      kReversePhaseDLateralTolM,
      kReversePhaseDYawTolRad);

  bool fwd_ok = !sim_f.empty() && CheckForwardSim(sim_f, length, width, forbidden, roads_vec, kForwardSimRoadsInflateM);

  if (!fwd_ok && res.transfer_polyline.size() >= 2) {
    auto ref_t = RefXYFromPolyline(res.transfer_polyline, kRefDsTransfer);
    const double tl_t = PolylineLength(res.transfer_polyline);
    double gx_t, gy_t, goal_yaw_t;
    PolylineTangentAt(res.transfer_polyline, std::max(0.0, tl_t - kPolylineTangentEndOffsetM), &gx_t, &gy_t, &goal_yaw_t);
    auto sim_t = TrackPathBicycle(
        ex,
        ey,
        yaw0,
        ref_t,
        false,
        kWheelbaseM,
        max_steer_rad,
        kMaxSteerRateRadS,
        dt_f,
        kFwdTSpeed,
        kFwdTLookahead,
        kFwdTStopTol,
        kFwdTYawTol,
        goal_yaw_t,
        kFwdTMaxSteps,
        false,
        kReverseSteerTaperEndFrac,
        false,
        kReversePhaseDLateralTolM,
        kReversePhaseDYawTolRad);

    if (sim_t.empty() || !CheckForwardSim(sim_t, length, width, forbidden, roads_vec, kForwardSimRoadsInflateM)) {
      res.sim_forward = sim_f;
      res.sim_message = sim_f.empty() ? "empty forward transfer" : "forward sim collision (transfer)";
      return;
    }

    auto ref_a = RefXYFromPolyline(res.forward_on_target_lane, kRefDsApproach);
    const double tl_a = PolylineLength(res.forward_on_target_lane);
    double gx_a, gy_a, goal_yaw_a;
    PolylineTangentAt(res.forward_on_target_lane, std::max(0.0, tl_a - kPolylineTangentEndOffsetM), &gx_a, &gy_a, &goal_yaw_a);
    const auto& lf0 = sim_t.back();
    auto sim_a = TrackPathBicycle(
        lf0.x,
        lf0.y,
        lf0.yaw,
        ref_a,
        false,
        kWheelbaseM,
        max_steer_rad,
        kMaxSteerRateRadS,
        dt_f,
        kFwdASpeed,
        kFwdALookahead,
        kFwdStopTol,
        kFwdYawTol,
        goal_yaw_a,
        kFwdTMaxSteps,
        false,
        kReverseSteerTaperEndFrac,
        false,
        kReversePhaseDLateralTolM,
        kReversePhaseDYawTolRad);

    if (sim_a.empty() || !CheckForwardSim(sim_a, length, width, forbidden, roads_vec, kForwardSimRoadsInflateM)) {
      res.sim_forward = sim_t;
      res.sim_message = sim_a.empty() ? "empty forward approach" : "forward sim collision (approach)";
      return;
    }

    const double t_base = sim_t.back().t + dt_f;
    sim_f = sim_t;
    for (size_t i = 0; i < sim_a.size(); ++i) {
      SimPose q = sim_a[i];
      q.t = t_base + static_cast<double>(i) * dt_f;
      sim_f.push_back(q);
    }
    fwd_ok = true;
  } else if (!fwd_ok) {
    res.sim_forward = sim_f;
    res.sim_message = sim_f.empty() ? "forward track empty" : "forward sim collision";
    return;
  }

  res.sim_forward = sim_f;
  const auto& lf = sim_f.back();
  const double slot_yaw = SlotYaw(res.target_row);
  const double gcx = res.slot.x + 0.5 * res.slot.w;
  const double gcy = res.slot.y + 0.55 * res.slot.h;

  // Respect planner-provided reverse reference.
  // For upper-last-slot-after-turn, enforce two-phase reverse:
  // (1) straight back to overshoot staging, then (2) normal turning reverse-in.
  std::vector<Point> reverse_plan = res.reverse_polyline;
  if (reverse_plan.size() < 2) {
    reverse_plan = MakeReverseBezierRef(lf.x, lf.y, gcx, gcy, slot_yaw, kReverseBezierSamples);
  }
  if (reverse_plan.size() >= 2) {
    size_t turn_start_idx = 0;
    double best_d = 1e18;
    for (size_t i = 0; i < reverse_plan.size(); ++i) {
      const double d = std::hypot(reverse_plan[i].x - res.staging.x, reverse_plan[i].y - res.staging.y);
      if (d < best_d) {
        best_d = d;
        turn_start_idx = i;
      }
    }
    if (turn_start_idx > 0 && best_d < 0.7) {
      reverse_plan = std::vector<Point>(
          reverse_plan.begin() + static_cast<std::ptrdiff_t>(turn_start_idx), reverse_plan.end());
    }
  }

  std::vector<SimPose> sim_r;
  if (upper_last_slot_after_turn && reverse_plan.size() >= 2) {
    // Use front-half criterion for overshoot hit:
    // start turning when vehicle front reaches overshoot (not rear/center).
    const Point overshoot_target = reverse_plan.front();
    const Point straight_goal_center{
        overshoot_target.x - 0.5 * length * std::cos(lf.yaw),
        overshoot_target.y - 0.5 * length * std::sin(lf.yaw)};
    std::vector<Point> rev_straight =
        SampleStraightSegment(Point{lf.x, lf.y}, straight_goal_center, 0.20);
    if (rev_straight.size() < 2) {
      rev_straight = std::vector<Point>{{lf.x, lf.y}, straight_goal_center};
    }
    auto rev_straight_ref = RefXYFromPolyline(rev_straight, kRefDsReverse);
    auto sim_r_pre = TrackPathBicycle(
        lf.x,
        lf.y,
        lf.yaw,
        rev_straight_ref,
        true,
        kWheelbaseM,
        max_steer_rad,
        kMaxSteerRateRadSReverse,
        kDtSim,
        0.65,
        0.90,
        0.12,
        0.55,
        lf.yaw,
        std::max(200, kReverseTrackMaxSteps / 3),
        false,
        kReverseSteerTaperEndFrac,
        false,
        kReversePhaseDLateralTolM,
        kReversePhaseDYawTolRad);
    if (sim_r_pre.empty()) {
      res.sim_message = "empty reverse pre-straight";
      return;
    }

    const auto& ls = sim_r_pre.back();
    std::vector<Point> rev_turn = MakeReverseBezierRef(ls.x, ls.y, gcx, gcy, slot_yaw, kReverseBezierSamples);
    auto rev_turn_ref = RefXYFromPolyline(rev_turn, kRefDsReverse);
    auto sim_r_turn = TrackPathBicycle(
        ls.x,
        ls.y,
        ls.yaw,
        rev_turn_ref,
        true,
        kWheelbaseM,
        max_steer_rad,
        kMaxSteerRateRadSReverse,
        kDtSim,
        kRevTargetSpeed,
        kRevLookahead,
        kRevStopTol,
        kRevYawTol,
        slot_yaw,
        kReverseTrackMaxSteps,
        true,
        kReverseSteerTaperEndFrac,
        true,
        kReversePhaseDLateralTolM,
        kReversePhaseDYawTolRad);
    if (sim_r_turn.empty()) {
      res.sim_reverse = sim_r_pre;
      res.sim_message = "empty reverse turn";
      return;
    }

    sim_r = sim_r_pre;
    const double t_base = sim_r.back().t + kDtSim;
    for (size_t i = 0; i < sim_r_turn.size(); ++i) {
      SimPose q = sim_r_turn[i];
      q.t = t_base + static_cast<double>(i) * kDtSim;
      sim_r.push_back(q);
    }
  } else {
    if (reverse_plan.size() < 2) {
      res.sim_message = "empty reverse";
      return;
    }
    const double d0 = std::hypot(lf.x - reverse_plan.front().x, lf.y - reverse_plan.front().y);
    if (d0 > 0.08) {
      std::vector<Point> bridge{{lf.x, lf.y}, reverse_plan.front()};
      reverse_plan = MergePolylines(bridge, reverse_plan);
    } else {
      reverse_plan.front() = Point{lf.x, lf.y};
    }
    auto rev_ref = RefXYFromPolyline(reverse_plan, kRefDsReverse);
    sim_r = TrackPathBicycle(
        lf.x,
        lf.y,
        lf.yaw,
        rev_ref,
        true,
        kWheelbaseM,
        max_steer_rad,
        kMaxSteerRateRadSReverse,
        kDtSim,
        kRevTargetSpeed,
        kRevLookahead,
        kRevStopTol,
        kRevYawTol,
        slot_yaw,
        kReverseTrackMaxSteps,
        true,
        kReverseSteerTaperEndFrac,
        true,
        kReversePhaseDLateralTolM,
        kReversePhaseDYawTolRad);
  }

  if (sim_r.empty()) {
    res.sim_message = "empty reverse";
    return;
  }
  const double reverse_obb_scale = upper_last_slot_after_turn ? 0.78 : kReverseSimObbScale;
  if (!CheckReverseSimLenient(sim_r, length, width, occ_and_obs, reverse_obb_scale)) {
    res.sim_reverse = sim_r;
    res.sim_message = "reverse sim collision";
    return;
  }

  res.sim_reverse = sim_r;
  const auto& lr = sim_r.back();
  const bool ok_slot = FinalInSlotLoose(lr.x, lr.y, lr.yaw, res.slot, slot_yaw, length, width);
  if (ok_slot) {
    res.sim_success = true;
    res.sim_message = "ok";
  } else {
    res.sim_success = false;
    res.sim_message = "final pose loose check failed";
  }
}

}  // namespace detail

bool ValidateForwardPolylineGeometry(const json& scene, PlanResult& res, double max_steer_deg) {
  return detail::ValidateForwardPolylineGeometryBody(scene, res, max_steer_deg);
}

void RunParkingSimulation(const json& scene, PlanResult& res, double max_steer_deg, double overshoot_m) {
  detail::RunParkingSimulationBody(scene, res, max_steer_deg, overshoot_m);
}

}  // namespace autopark
