// =============================================================================
// Module: main (executable entry)
// -----------------------------------------------------------------------------
// Purpose:
//   Command-line orchestration: load scene JSON, run geometric planning
//   (lane transfer + forward overshoot + reverse Bezier), optional kinematic
//   simulation, emit result JSON and/or SVG; supports batch visualization over
//   all free slots.
//
// Inputs:
//   - argv flags (--map, --out, --plot-out, steering/overshoot, --batch-viz,
//     --plan-row0, etc.) and files on disk.
//
// Outputs:
//   - stdout / files: parking_result-style JSON, trajectory SVG, batch summary.
//   - Process exit codes from plan/sim/batch success counts.
//
// Calls (dependencies):
//   - geometry_lane, connector_transfer, forward_overshoot, reverse_bezier,
//     parking_simulate (Validate*, RunParkingSimulation), bicycle_sim (PolylineLength
//     for messages/JSON), viz_svg (SaveParkingPlotSvg, SampleVehicleAlongPolyline),
//     sim_tune, types (ap::).
// =============================================================================

#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

#include "autopark/bicycle_sim.hpp"
#include "autopark/connector_transfer.hpp"
#include "autopark/forward_overshoot.hpp"
#include "autopark/geometry_lane.hpp"
#include "autopark/parking_simulate.hpp"
#include "autopark/reverse_bezier.hpp"
#include "autopark/sim_tune.hpp"
#include "autopark/types.hpp"
#include "autopark/viz_svg.hpp"

using json = nlohmann::json;

namespace {

// =============================================================================
// Module: CLI configuration
// -----------------------------------------------------------------------------
// Default steering / geometry limits and the Args struct: input/output paths,
// planner parameters, and feature flags (plot, batch, plan-row0). Populated by
// ParseArgs; consumed by main and batch driver.
// =============================================================================

constexpr double kDefaultMaxSteerDeg = 35.0;
constexpr double kDefaultOvershootM = 4.8;
constexpr double kDefaultNearM = 2.5;
struct Args {
  std::string map{"data/parking_scene.json"};
  std::string out{"outputs/parking_result_cpp.json"};
  std::string plot_out{"outputs/parking_plot_cpp.svg"};
  /// Optional sensor frame JSON; if provided it is injected into scene["sensor_frame"].
  std::string sensor_frame{};
  double max_steer_deg{kDefaultMaxSteerDeg};
  double overshoot_m{kDefaultOvershootM};
  double near_m{kDefaultNearM};
  /// If set, reverse collision uses sensor obstacles only (no map occupied/obstacles fallback).
  bool sensor_replace_map_obstacles{false};
  bool plan_row0{false};
  bool no_plot{false};
  bool show_plot{false};
  /// When set: for each unoccupied slot run plan + sim and write one SVG plus batch summary.
  bool batch_viz{false};
  std::string batch_out_dir{"outputs/batch_viz"};
  /// Interactive text mode to configure row_count/slot_count/target and run once.
  bool tui{false};
  std::string custom_map_out{"outputs/custom_map.json"};
};

// =============================================================================
// Module: Geometric planning (single target)
// -----------------------------------------------------------------------------
// Reads scene JSON (ego, parking, roads, target_slots, occupied_slots) and
// fills PlanResult: slot rect, optional inter-lane transfer polyline, forward
// path with near/staging (overshoot), merged forward polyline, forward-path
// validation, and reverse Bezier reference. Does not run bicycle simulation.
// =============================================================================

ap::PlanResult PlanAndBuildReference(
    const json& data,
    double max_steer_deg,
    double overshoot_m,
    double near_m) {
  ap::PlanResult res;
  const json& cfg = data.at("parking");
  const json roads = data.value("roads", json::array());
  const json ego = data.at("ego_vehicle");
  const double ex = ego.at("x").get<double>();
  const double ey = ego.at("y").get<double>();
  const double length = ego.value("length", 4.8);
  double map_w = data.value("map_width", 75.0);
  if (data.contains("map") && data.at("map").is_object()) {
    map_w = data.at("map").value("width", map_w);
  }
  const auto targets = data.value("target_slots", json::array());
  if (targets.empty()) {
    res.message = "target_slots is empty";
    return res;
  }

  const int row = targets[0].at("row").get<int>();
  const int idx = targets[0].at("index").get<int>();
  res.target_row = row;
  res.target_idx = idx;
  res.slot = ap::SlotRect(cfg, row, idx);

  for (const auto& o : data.value("occupied_slots", json::array())) {
    if (o.at("row").get<int>() == row && o.at("index").get<int>() == idx) {
      res.message = "target slot is occupied";
      return res;
    }
  }

  const std::string ego_g = ap::EgoLaneGroup(ex, ey, roads);
  const std::string tgt_g = ap::TargetLaneGroup(row);
  const auto transfer_opt = ap::BuildInterLaneTransfer(ex, ey, ego_g, tgt_g, roads);
  if (!transfer_opt.has_value()) {
    res.message = "failed to build inter-lane transfer";
    return res;
  }
  std::vector<ap::Point> transfer = *transfer_opt;
  const ap::Point start_on_target = transfer.empty() ? ap::Point{ex, ey} : transfer.back();

  const double slot_cx = res.slot.x + 0.5 * res.slot.w;
  const double lane_y = ap::LaneYForRow(row);

  const auto fwd_built_target = ap::BuildForwardPathOvershoot(
      start_on_target.x,
      start_on_target.y,
      slot_cx,
      lane_y,
      row,
      map_w,
      overshoot_m,
      near_m,
      roads,
      length);
  if (!fwd_built_target.has_value()) {
    res.message = "failed to build approach/overshoot path";
    return res;
  }

  auto [fwd_raw_target, approach_near_target, staging_target] = *fwd_built_target;
  auto fwd_raw = fwd_raw_target;
  auto approach_near = approach_near_target;
  auto staging = staging_target;

  const int slot_count = cfg.at("slot_count").get<int>();
  const bool upper_last_slot_after_turn = (row != 2 && row != 3) && (idx == slot_count - 1) && !transfer.empty();
  if (upper_last_slot_after_turn && idx > 0) {
    // Extension stage for the first slot after connector turn:
    // drive forward to the second slot's overshoot first, then reverse straight
    // back to the original target overshoot before normal reverse turning.
    const auto second_slot = ap::SlotRect(cfg, row, idx - 1);
    const double second_slot_cx = second_slot.x + 0.5 * second_slot.w;
    const auto fwd_built_second = ap::BuildForwardPathOvershoot(
        start_on_target.x,
        start_on_target.y,
        second_slot_cx,
        lane_y,
        row,
        map_w,
        overshoot_m,
        near_m,
        roads,
        length);
    if (fwd_built_second.has_value()) {
      auto [fwd_raw_second, approach_near_second, staging_second] = *fwd_built_second;
      fwd_raw = std::move(fwd_raw_second);
      approach_near = approach_near_second;
      staging = staging_second;
    }
  }
  res.approach_near = approach_near;
  res.staging = staging;
  res.transfer_polyline = transfer;
  res.forward_on_target_lane = fwd_raw;
  res.forward_polyline = ap::MergePolylines(transfer, fwd_raw);

  if (!ap::ValidateForwardPolylineGeometry(data, res, max_steer_deg)) {
    res.message = "forward path geometry vs roads/forbidden (collision check)";
    return res;
  }

  const double slot_yaw = ap::SlotYaw(row);
  const double gx = res.slot.x + 0.5 * res.slot.w;
  const double gy = res.slot.y + 0.55 * res.slot.h;
  if (upper_last_slot_after_turn && idx > 0) {
    std::vector<ap::Point> reverse_straight{{staging.x, staging.y}, {staging_target.x, staging_target.y}};
    auto reverse_turn = ap::MakeReverseBezierRef(
        staging_target.x,
        staging_target.y,
        gx,
        gy,
        slot_yaw,
        ap::sim_tune::kReverseBezierSamples);
    res.staging = staging_target;
    res.reverse_polyline = ap::MergePolylines(reverse_straight, reverse_turn);
  } else {
    res.reverse_polyline = ap::MakeReverseBezierRef(
        staging.x, staging.y, gx, gy, slot_yaw, ap::sim_tune::kReverseBezierSamples);
  }

  const double fwd_dist = ap::PolylineLength(res.forward_polyline);
  const double rev_dist = ap::PolylineLength(res.reverse_polyline);
  std::ostringstream oss;
  oss << "planned: fwd=" << std::round(fwd_dist * 1000.0) / 1000.0 << " m, rev="
      << std::round(rev_dist * 1000.0) / 1000.0 << " m, |steer|<=" << max_steer_deg << " deg";
  res.message = oss.str();
  res.success = true;
  return res;
}

// =============================================================================
// Module: Result JSON serialization
// -----------------------------------------------------------------------------
// Builds the user-facing result object: target/ego metadata, slot bounds,
// polylines, arc lengths, success/sim flags, sparse vehicle poses along
// references (for viz/JSON consumers), and full sim_forward / sim_reverse arrays.
// =============================================================================

json ToResultJson(const json& scene, const ap::PlanResult& res, double max_steer_deg, double overshoot_m) {
  const auto ego = scene.at("ego_vehicle");
  json out;
  out["target"] = {{"row", res.target_row}, {"index", res.target_idx}};
  out["ego_start"] = {
      {"x", ego.at("x").get<double>()},
      {"y", ego.at("y").get<double>()},
      {"yaw", ego.value("theta", ego.value("yaw", 0.0))},
  };
  out["slot_yaw"] = ap::SlotYaw(res.target_row);
  out["vehicle"] = {
      {"length", ego.value("length", 4.8)},
      {"width", ego.value("width", 2.0)},
  };
  out["max_steer_deg"] = max_steer_deg;
  out["wheelbase_m"] = ap::sim_tune::kWheelbaseM;
  out["overshoot_m"] = overshoot_m;
  out["approach_near"] = {{"x", res.approach_near.x}, {"y", res.approach_near.y}};
  out["staging"] = {{"x", res.staging.x}, {"y", res.staging.y}};
  out["slot_bounds"] = {{"x", res.slot.x}, {"y", res.slot.y}, {"w", res.slot.w}, {"h", res.slot.h}};
  {
    const double fwd_d = ap::PolylineLength(res.forward_polyline);
    const double rev_d = ap::PolylineLength(res.reverse_polyline);
    out["forward_distance_m"] = std::round(fwd_d * 1000.0) / 1000.0;
    out["reverse_distance_m"] = std::round(rev_d * 1000.0) / 1000.0;
    out["total_drive_distance_m"] = std::round((fwd_d + rev_d) * 1000.0) / 1000.0;
  }
  out["success"] = res.success;
  out["sim_success"] = res.sim_success;
  out["message"] = res.message;
  out["sim_message"] = res.sim_message.empty() ? (res.success ? std::string("ok") : std::string("planning failed")) : res.sim_message;
  out["collision_model_debug"] = res.collision_model_debug;
  out["forward_polyline"] = json::array();
  for (const auto& p : res.forward_polyline) out["forward_polyline"].push_back({{"x", p.x}, {"y", p.y}});
  out["reverse_polyline"] = json::array();
  for (const auto& p : res.reverse_polyline) out["reverse_polyline"].push_back({{"x", p.x}, {"y", p.y}});
  constexpr double kVizStep = 0.45;
  constexpr int kVizStride = 50;
  out["viz_densify_step_m"] = kVizStep;
  out["viz_body_sample_stride"] = kVizStride;
  out["forward_body_samples"] = json::array();
  for (const auto& s : ap::SampleVehicleAlongPolyline(res.forward_polyline, false, kVizStep, kVizStride)) {
    out["forward_body_samples"].push_back({{"x", s.x}, {"y", s.y}, {"yaw", s.yaw}});
  }
  out["reverse_body_samples"] = json::array();
  for (const auto& s : ap::SampleVehicleAlongPolyline(res.reverse_polyline, true, kVizStep, kVizStride)) {
    out["reverse_body_samples"].push_back({{"x", s.x}, {"y", s.y}, {"yaw", s.yaw}});
  }
  out["sim_forward"] = json::array();
  for (const auto& p : res.sim_forward) {
    json o;
    o["t"] = p.t;
    o["x"] = p.x;
    o["y"] = p.y;
    o["yaw"] = p.yaw;
    o["v"] = p.v;
    o["vx"] = p.vx;
    o["vy"] = p.vy;
    o["steer"] = p.steer;
    o["steer_deg"] = p.steer * 180.0 / 3.14159265358979323846;
    o["reverse_mode"] = p.reverse_mode;
    o["reverse_phase_d"] = p.reverse_phase_d;
    out["sim_forward"].push_back(std::move(o));
  }
  out["sim_reverse"] = json::array();
  for (const auto& p : res.sim_reverse) {
    json o;
    o["t"] = p.t;
    o["x"] = p.x;
    o["y"] = p.y;
    o["yaw"] = p.yaw;
    o["v"] = p.v;
    o["vx"] = p.vx;
    o["vy"] = p.vy;
    o["steer"] = p.steer;
    o["steer_deg"] = p.steer * 180.0 / 3.14159265358979323846;
    o["reverse_mode"] = p.reverse_mode;
    o["reverse_phase_d"] = p.reverse_phase_d;
    out["sim_reverse"].push_back(std::move(o));
  }
  return out;
}

// =============================================================================
// Module: Command-line parsing
// -----------------------------------------------------------------------------
// Walks argv, fills Args, prints usage on --help (returns nullopt → exit 0),
// reports unknown/missing options on stderr.
// =============================================================================

std::optional<Args> ParseArgs(int argc, char** argv) {
  Args a;
  for (int i = 1; i < argc; ++i) {
    const std::string k = argv[i];
    auto need = [&](const char* name) -> std::optional<std::string> {
      if (i + 1 >= argc) {
        std::cerr << "[error] missing value for " << name << "\n";
        return std::nullopt;
      }
      return std::string(argv[++i]);
    };
    if (k == "--map") {
      auto v = need("--map");
      if (!v.has_value()) return std::nullopt;
      a.map = *v;
    } else if (k == "--out") {
      auto v = need("--out");
      if (!v.has_value()) return std::nullopt;
      a.out = *v;
    } else if (k == "--max-steer-deg") {
      auto v = need("--max-steer-deg");
      if (!v.has_value()) return std::nullopt;
      a.max_steer_deg = std::stod(*v);
    } else if (k == "--overshoot-m") {
      auto v = need("--overshoot-m");
      if (!v.has_value()) return std::nullopt;
      a.overshoot_m = std::stod(*v);
    } else if (k == "--near-m") {
      auto v = need("--near-m");
      if (!v.has_value()) return std::nullopt;
      a.near_m = std::stod(*v);
    } else if (k == "--sensor-frame") {
      auto v = need("--sensor-frame");
      if (!v.has_value()) return std::nullopt;
      a.sensor_frame = *v;
    } else if (k == "--sensor-replace-map-obstacles") {
      a.sensor_replace_map_obstacles = true;
    } else if (k == "--plan-row0") {
      a.plan_row0 = true;
    } else if (k == "--plot-out") {
      auto v = need("--plot-out");
      if (!v.has_value()) return std::nullopt;
      a.plot_out = *v;
    } else if (k == "--no-plot") {
      a.no_plot = true;
    } else if (k == "--show-plot") {
      a.show_plot = true;
    } else if (k == "--batch-viz") {
      a.batch_viz = true;
    } else if (k == "--batch-out-dir") {
      auto v = need("--batch-out-dir");
      if (!v.has_value()) return std::nullopt;
      a.batch_out_dir = *v;
    } else if (k == "--tui") {
      a.tui = true;
    } else if (k == "--custom-map-out") {
      auto v = need("--custom-map-out");
      if (!v.has_value()) return std::nullopt;
      a.custom_map_out = *v;
    } else if (k == "-h" || k == "--help") {
      std::cout
          << "autopark_run options:\n"
          << "  --map <path>            scene json path (default: data/parking_scene.json; cwd should be autopark_cpp/)\n"
          << "  --out <path>            output result json (default: outputs/parking_result_cpp.json)\n"
          << "  --plot-out <path>       trajectory SVG (default: outputs/parking_plot_cpp.svg)\n"
          << "  --no-plot               skip SVG generation\n"
          << "  --show-plot             open SVG with default viewer after save\n"
          << "  --max-steer-deg <deg>   max steering angle\n"
          << "  --overshoot-m <m>       overshoot distance\n"
          << "  --near-m <m>            near-point offset\n"
          << "  --sensor-frame <path>   sensor frame json (array or {detections:[...]})\n"
          << "  --sensor-replace-map-obstacles  use sensor obstacles only in reverse collision check\n"
          << "  --plan-row0             override target with first free slot in row0\n"
          << "  --batch-viz             all unoccupied slots: plan+sim each, write SVG + batch summary.json\n"
          << "  --batch-out-dir <path>  batch SVG/summary directory (default: outputs/batch_viz)\n"
          << "  --tui                   interactive TUI main menu (configure map/target and run)\n"
          << "  --custom-map-out <path> TUI-generated map json path (default: outputs/custom_map.json)\n"
          << "Note: default paths assume current working directory is the autopark_cpp/ project folder.\n";
      return std::nullopt;
    } else {
      std::cerr << "[error] unknown option: " << k << "\n";
      return std::nullopt;
    }
  }
  return a;
}

// =============================================================================
// Module: Filesystem & viewer helpers
// -----------------------------------------------------------------------------
// Cross-platform: open SVG with default app; create parent directories for
// output paths via shell one-liners (best-effort).
// =============================================================================

bool OpenPlotWithDefaultViewer(const std::string& path) {
#ifdef _WIN32
  const std::string cmd = "cmd /c start \"\" \"" + path + "\"";
#else
  const std::string cmd = "xdg-open \"" + path + "\"";
#endif
  return std::system(cmd.c_str()) == 0;
}

void EnsureParentDir(const std::string& p) {
  const auto pos = p.find_last_of("/\\");
  if (pos == std::string::npos) return;
  const std::string dir = p.substr(0, pos);
  if (dir.empty()) return;
#ifdef _WIN32
  const std::string cmd = "if not exist \"" + dir + "\" mkdir \"" + dir + "\"";
#else
  const std::string cmd = "mkdir -p \"" + dir + "\"";
#endif
  std::ignore = std::system(cmd.c_str());
}

bool PromptIntInRange(const std::string& label, int lo, int hi, int& out) {
  while (true) {
    std::cout << label;
    int v = 0;
    if (std::cin >> v) {
      if (v >= lo && v <= hi) {
        out = v;
        return true;
      }
      std::cout << "[error] value out of range (" << lo << "..." << hi << ")\n";
    } else {
      if (std::cin.eof()) return false;
      std::cout << "[error] invalid integer input\n";
    }
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
}

json BuildAutoTargetPoseFromSlot(const json& parking_cfg, int row, int idx) {
  const ap::Rect slot = ap::SlotRect(parking_cfg, row, idx);
  const double target_x = slot.x + 0.5 * slot.w;
  const double target_y = slot.y + 0.55 * slot.h;
  const double target_theta = ap::SlotYaw(row);
  return json{
      {"x", target_x},
      {"y", target_y},
      {"theta", target_theta},
      {"row", row},
      {"index", idx},
      {"auto_generated", true},
  };
}

void AutoExpandMapAndRoadsForSlotCount(json& scene, int slot_count) {
  const json parking_cfg = scene.value("parking", json::object());
  const double start_x = parking_cfg.value("start_x", 12.0);
  const double slot_w = parking_cfg.value("slot_width", 4.8);
  const double required_slot_right = start_x + static_cast<double>(slot_count) * slot_w;
  const double connector_gap = 0.2;
  const double map_right_margin = 3.0;

  double map_w = scene.value("map_width", 75.0);
  if (scene.contains("map") && scene.at("map").is_object()) {
    map_w = scene.at("map").value("width", map_w);
  }
  double connector_w = 6.0;
  if (scene.contains("roads") && scene.at("roads").is_array()) {
    for (const auto& rd : scene.at("roads")) {
      if (rd.value("id", "") == "right_connector") {
        connector_w = rd.value("w", connector_w);
        break;
      }
    }
  }
  const double connector_x = required_slot_right + connector_gap;
  const double target_map_w = std::max(map_w, connector_x + connector_w + map_right_margin);
  scene["map_width"] = target_map_w;
  scene["map"]["width"] = target_map_w;

  if (!scene.contains("roads") || !scene.at("roads").is_array()) {
    return;
  }
  json roads = scene.at("roads");
  for (auto& rd : roads) {
    const std::string id = rd.value("id", "");
    if (id == "lower_main_lane" || id == "upper_main_lane") {
      rd["x"] = 0.0;
      rd["w"] = target_map_w;
    } else if (id == "right_connector") {
      rd["x"] = connector_x;
    }
  }
  scene["roads"] = roads;
}

bool ConfigureSceneWithTui(json& scene, Args& args) {
  std::cout << "\n=== Autopark TUI (Custom Map) ===\n";
  std::cout << "Main menu (loop): configure map and target slot, then run.\n";

  int row_count = scene.value("parking", json::object()).value("row_count", 4);
  row_count = static_cast<int>(ap::Clamp(static_cast<double>(row_count), 1.0, 4.0));
  int slot_count = scene.value("parking", json::object()).value("slot_count", 10);
  slot_count = static_cast<int>(ap::Clamp(static_cast<double>(slot_count), 1.0, 20.0));

  int target_row = 0;
  int target_idx = 0;
  if (scene.contains("target_slots") && scene.at("target_slots").is_array() && !scene.at("target_slots").empty()) {
    target_row = scene.at("target_slots")[0].value("row", 0);
    target_idx = scene.at("target_slots")[0].value("index", 0);
  }
  target_row = static_cast<int>(ap::Clamp(static_cast<double>(target_row), 0.0, static_cast<double>(row_count - 1)));
  target_idx = static_cast<int>(ap::Clamp(static_cast<double>(target_idx), 0.0, static_cast<double>(slot_count - 1)));

  while (true) {
    std::cout << "\n--- TUI Menu ---\n";
    std::cout << "1) Set row count (1-4)\n";
    std::cout << "2) Set slots per row (1-20)\n";
    std::cout << "3) Set target slot (row, index)\n";
    std::cout << "4) Show current settings\n";
    std::cout << "5) Run planning and export files\n";
    std::cout << "0) Exit TUI\n";
    int menu = -1;
    if (!PromptIntInRange("Select menu: ", 0, 5, menu)) return false;
    if (menu == 0) return false;

    if (menu == 1) {
      if (!PromptIntInRange("Row count (1-4): ", 1, 4, row_count)) return false;
      target_row = static_cast<int>(ap::Clamp(static_cast<double>(target_row), 0.0, static_cast<double>(row_count - 1)));
      continue;
    }
    if (menu == 2) {
      if (!PromptIntInRange("Slots per row (1-20): ", 1, 20, slot_count)) return false;
      target_idx = static_cast<int>(ap::Clamp(static_cast<double>(target_idx), 0.0, static_cast<double>(slot_count - 1)));
      continue;
    }
    if (menu == 3) {
      if (!PromptIntInRange("Target row (0-based): ", 0, row_count - 1, target_row)) return false;
      if (!PromptIntInRange("Target index (0-based): ", 0, slot_count - 1, target_idx)) return false;
      continue;
    }

    json preview = scene;
    preview["parking"]["row_count"] = row_count;
    preview["parking"]["slot_count"] = slot_count;
    AutoExpandMapAndRoadsForSlotCount(preview, slot_count);
    const json target_pose = BuildAutoTargetPoseFromSlot(preview.at("parking"), target_row, target_idx);

    const double start_x = preview.at("parking").value("start_x", 12.0);
    const double slot_w = preview.at("parking").value("slot_width", 4.8);
    const double last_slot_right = start_x + static_cast<double>(slot_count) * slot_w;
    double connector_x = 0.0;
    for (const auto& rd : preview.value("roads", json::array())) {
      if (rd.value("id", "") == "right_connector") {
        connector_x = rd.value("x", connector_x);
        break;
      }
    }
    const double map_w = preview.value("map_width", preview.value("map", json::object()).value("width", 75.0));

    if (menu == 4) {
      std::cout << "[info] row_count=" << row_count << ", slot_count=" << slot_count
                << ", target=(" << target_row << "," << target_idx << ")\n";
      std::cout << "[info] target pose(auto): x=" << target_pose.at("x").get<double>()
                << ", y=" << target_pose.at("y").get<double>()
                << ", theta(rad)=" << target_pose.at("theta").get<double>() << "\n";
      std::cout << "[info] geometry summary: last_slot_right=" << last_slot_right
                << ", connector_x=" << connector_x
                << ", map_width=" << map_w << "\n";
      continue;
    }

    scene = std::move(preview);
    scene["target_slots"] = json::array({{{"row", target_row}, {"index", target_idx}, {"label", "TUI Goal"}}});
    scene["target_pose"] = target_pose;
    scene["occupied_slots"] = json::array();

    std::ostringstream out_json;
    out_json << "outputs/custom_result_r" << target_row << "_i" << target_idx << ".json";
    args.out = out_json.str();
    std::ostringstream out_svg;
    out_svg << "outputs/custom_plot_r" << target_row << "_i" << target_idx << ".svg";
    args.plot_out = out_svg.str();

    EnsureParentDir(args.custom_map_out);
    std::ofstream mf(args.custom_map_out);
    if (!mf) {
      std::cerr << "[error] cannot write TUI map file: " << args.custom_map_out << "\n";
      return false;
    }
    mf << scene.dump(2) << "\n";

    std::cout << "[info] target pose auto-generated from slot: x=" << target_pose.at("x").get<double>()
              << ", y=" << target_pose.at("y").get<double>()
              << ", theta(rad)=" << target_pose.at("theta").get<double>() << "\n";
    std::cout << "[info] geometry summary: last_slot_right=" << last_slot_right
              << ", connector_x=" << connector_x
              << ", map_width=" << map_w << "\n";
    std::cout << "[ok] wrote custom map: " << args.custom_map_out << "\n";
    std::cout << "[ok] output json: " << args.out << "\n";
    std::cout << "[ok] output svg : " << args.plot_out << "\n";
    return true;
  }
}

bool InjectSensorFrameIntoScene(const Args& args, json& scene) {
  json sensor_cfg = scene.value("sensor_collision", json::object());
  bool touched = false;

  if (!args.sensor_frame.empty()) {
    std::ifstream sfin(args.sensor_frame);
    if (!sfin) {
      std::cerr << "[error] sensor frame file not found: " << args.sensor_frame << "\n";
      return false;
    }
    json frame;
    try {
      sfin >> frame;
    } catch (const std::exception& e) {
      std::cerr << "[error] failed to parse sensor frame json: " << e.what() << "\n";
      return false;
    }

    if (frame.is_array()) {
      scene["sensor_detections"] = frame;
    } else if (frame.is_object() && frame.contains("detections") && frame.at("detections").is_array()) {
      scene["sensor_frame"] = frame;
      scene["sensor_detections"] = frame.at("detections");
    } else {
      std::cerr << "[error] invalid sensor frame format. expected array or object with detections[]\n";
      return false;
    }
    sensor_cfg["enabled"] = true;
    touched = true;
  }

  if (args.sensor_replace_map_obstacles) {
    sensor_cfg["replace_map_obstacles"] = true;
    sensor_cfg["enabled"] = true;
    touched = true;
  }

  if (touched) {
    scene["sensor_collision"] = sensor_cfg;
  }
  return true;
}

// =============================================================================
// Module: Scene target override (--plan-row0)
// -----------------------------------------------------------------------------
// Rewrites target_slots to the first unoccupied index in row 0 for quick tests.
// =============================================================================

void OverrideTargetRow0(json& scene) {
  const int slot_count = scene.at("parking").at("slot_count").get<int>();
  std::set<int> occ_idx;
  for (const auto& o : scene.value("occupied_slots", json::array())) {
    if (o.at("row").get<int>() == 0) occ_idx.insert(o.at("index").get<int>());
  }
  for (int i = 0; i < slot_count; ++i) {
    if (!occ_idx.count(i)) {
      scene["target_slots"] = json::array({{{"row", 0}, {"index", i}, {"label", "plan_row0_cpp"}}});
      return;
    }
  }
}

// =============================================================================
// Module: Batch visualization driver
// -----------------------------------------------------------------------------
// For every unoccupied (row, index): clone scene, set target, run geometric
// plan + RunParkingSimulation, write one SVG per slot, append a record to
// summary JSON. Returns failure count (or -1 if summary file cannot be written).
// =============================================================================

int RunBatchVizAllSlots(const json& scene_base, const Args& args) {
  std::set<std::pair<int, int>> occ;
  for (const auto& o : scene_base.value("occupied_slots", json::array())) {
    occ.insert({o.at("row").get<int>(), o.at("index").get<int>()});
  }
  const int sc = scene_base.at("parking").at("slot_count").get<int>();
  json summary = json::array();
  int tested = 0;
  int failed = 0;

  for (int row = 0; row < 4; ++row) {
    for (int idx = 0; idx < sc; ++idx) {
      if (occ.count({row, idx}) > 0) {
        continue;
      }
      ++tested;
      json scene = scene_base;
      std::ostringstream label;
      label << "batch_viz_r" << row << "_i" << idx;
      scene["target_slots"] =
          json::array({{{"row", row}, {"index", idx}, {"label", label.str()}}});

      ap::PlanResult plan = PlanAndBuildReference(scene, args.max_steer_deg, args.overshoot_m, args.near_m);
      if (plan.success) {
        ap::RunParkingSimulation(scene, plan, args.max_steer_deg, args.overshoot_m);
        plan.success = plan.sim_success;
        if (!plan.sim_success && !plan.sim_message.empty()) {
          plan.message = plan.sim_message;
        }
      }

      std::ostringstream svg_name;
      svg_name << args.batch_out_dir << "/viz_r" << row << "_i" << idx << ".svg";
      const std::string svg_path = svg_name.str();
      EnsureParentDir(svg_path);
      const bool svg_ok =
          ap::SaveParkingPlotSvg(scene, plan, args.max_steer_deg, args.overshoot_m, svg_path);
      if (!svg_ok) {
        std::cerr << "[warn] batch: failed to write SVG " << svg_path << "\n";
      }

      json rec;
      rec["row"] = row;
      rec["index"] = idx;
      rec["success"] = plan.success;
      rec["sim_success"] = plan.sim_success;
      rec["message"] = plan.message;
      rec["sim_message"] = plan.sim_message;
      rec["collision_model_debug"] = plan.collision_model_debug;
      rec["svg"] = svg_path;
      rec["svg_ok"] = svg_ok;
      summary.push_back(std::move(rec));

      if (!plan.success || !svg_ok) {
        ++failed;
      }
    }
  }

  const std::string summary_path = args.batch_out_dir + "/summary.json";
  EnsureParentDir(summary_path);
  json root;
  root["map"] = args.map;
  root["batch_out_dir"] = args.batch_out_dir;
  root["free_slots_tested"] = tested;
  root["failed_count"] = failed;
  root["cases"] = std::move(summary);
  std::ofstream sf(summary_path);
  if (sf) {
    sf << root.dump(2) << "\n";
    std::cout << "[batch-viz] wrote " << summary_path << " (" << tested << " slots, " << failed << " failed)\n";
  } else {
    std::cerr << "[error] batch: cannot write " << summary_path << "\n";
    return -1;
  }
  return failed;
}

}  // namespace

// =============================================================================
// Module: Program entry
// -----------------------------------------------------------------------------
// Load scene → optional batch branch (--batch-viz) → else single-target flow:
// optional row0 override, plan, simulate, write JSON, optional SVG and viewer.
// Exit codes: 0 success, 1 I/O or planning/sim/batch failures as applicable.
// =============================================================================

int main(int argc, char** argv) {
  const auto args_opt = ParseArgs(argc, argv);
  if (!args_opt.has_value()) return 0;
  Args args = *args_opt;

  // --- Load scene JSON from --map ---
  std::ifstream fin(args.map);
  if (!fin) {
    std::cerr << "[error] scene file not found: " << args.map << "\n";
    return 1;
  }

  json scene;
  try {
    fin >> scene;
  } catch (const std::exception& e) {
    std::cerr << "[error] failed to parse json: " << e.what() << "\n";
    return 1;
  }

  if (!InjectSensorFrameIntoScene(args, scene)) {
    return 1;
  }

  if (args.tui) {
    if (!ConfigureSceneWithTui(scene, args)) {
      std::cerr << "[info] TUI canceled.\n";
      return 1;
    }
  }

  // --- Batch mode: all free slots → SVGs + summary.json ---
  if (args.batch_viz) {
    if (args.plan_row0) {
      std::cerr << "[warn] --plan-row0 ignored when using --batch-viz\n";
    }
    EnsureParentDir(args.batch_out_dir + "/summary.json");
    const int fail = RunBatchVizAllSlots(scene, args);
    if (fail < 0) {
      return 1;
    }
    return fail > 0 ? 1 : 0;
  }

  // --- Single-target: optional row0 override, plan, simulate, JSON + SVG ---
  if (args.plan_row0) OverrideTargetRow0(scene);
  ap::PlanResult plan = PlanAndBuildReference(scene, args.max_steer_deg, args.overshoot_m, args.near_m);
  if (plan.success) {
    ap::RunParkingSimulation(scene, plan, args.max_steer_deg, args.overshoot_m);
    plan.success = plan.sim_success;
    if (!plan.sim_success && !plan.sim_message.empty()) {
      plan.message = plan.sim_message;
    }
  }
  const json out = ToResultJson(scene, plan, args.max_steer_deg, args.overshoot_m);

  // --- Write result JSON to --out and echo to stdout ---
  EnsureParentDir(args.out);
  std::ofstream fout(args.out);
  if (!fout) {
    std::cerr << "[error] cannot write output file: " << args.out << "\n";
    return 1;
  }
  fout << out.dump(2) << "\n";
  std::cout << out.dump(2) << "\n";
  std::cout << "[ok] wrote " << args.out << "\n";

  // --- Optional trajectory SVG (--plot-out); --show-plot opens viewer ---
  if (!args.no_plot) {
    EnsureParentDir(args.plot_out);
    if (ap::SaveParkingPlotSvg(scene, plan, args.max_steer_deg, args.overshoot_m, args.plot_out)) {
      std::cout << "[ok] wrote plot " << args.plot_out << "\n";
      if (args.show_plot) {
        if (!OpenPlotWithDefaultViewer(args.plot_out)) {
          std::cerr << "[warn] could not open plot viewer\n";
        }
      }
    } else {
      std::cerr << "[warn] failed to write SVG plot\n";
    }
  }

  return plan.success ? 0 : 1;
}
