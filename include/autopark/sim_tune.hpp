#pragma once

// =============================================================================
// Module: sim_tune (tuning constants)
// -----------------------------------------------------------------------------
// Purpose:
//   Centralizes numeric thresholds and gains for Pure Pursuit / bicycle
//   tracking, forward and reverse simulation steps, resampling, and planner-
//   side geometry (Bezier samples, polyline tangents, collision padding).
//
// Inputs / outputs:
//   - No functions; `inline constexpr` values are read by call sites.
//
// Calls (dependencies):
//   - None (header-only constants in namespace autopark::sim_tune).
//
// Used by:
//   - bicycle_sim, parking_simulate, main (and any code referencing ap::sim_tune).
// =============================================================================

namespace autopark::sim_tune {

inline constexpr double kNearGoalShrinkDistM = 4.0;
inline constexpr double kNearGoalLdEffMin = 0.32;
inline constexpr double kNearGoalSpeedAbsMin = 0.35;
inline constexpr double kReverseLookaheadFloorM = 0.58;
inline constexpr double kReversePhaseDExitYawFactor = 2.2;
inline constexpr double kReversePhaseDExitLatFactor = 1.45;

inline constexpr double kMaxAccelMs2 = 6.0;
inline constexpr double kReverseSteerNearGoalM = 7.0;
inline constexpr double kReverseStopCenterXTolM = 0.5;
inline constexpr double kReverseStopCenterYTolM = 0.05;

inline constexpr double kWheelbaseM = 2.7;
inline constexpr double kMaxSteerRateRadS = 1.2;
inline constexpr double kMaxSteerRateRadSReverse = 1.2;
inline constexpr double kReverseSteerTaperEndFrac = 0.42;
inline constexpr int kReverseTrackMaxSteps = 600;
inline constexpr double kReversePhaseDLateralTolM = 3.0;
inline constexpr double kReversePhaseDYawTolRad = 0.03;
inline constexpr double kForwardSimRoadsInflateM = 0.38;
inline constexpr double kReverseSimObbScale = 0.88;
inline constexpr double kFinalSlotGeomPadM = 2.0;
inline constexpr int kFinalSlotMinCornersInPadded = 1;

inline constexpr double kDtSim = 0.05;

inline constexpr double kRefDsForwardFull = 0.30;
inline constexpr double kFwdSpeed = 1.55;
inline constexpr double kFwdLookahead = 1.85;
inline constexpr double kFwdStopTol = 0.55;
inline constexpr double kFwdYawTol = 0.22;
inline constexpr int kFwdMaxSteps = 14000;

inline constexpr double kRefDsTransfer = 0.18;
inline constexpr double kFwdTSpeed = 1.05;
inline constexpr double kFwdTLookahead = 1.45;
inline constexpr double kFwdTStopTol = 0.55;
inline constexpr double kFwdTYawTol = 0.32;
inline constexpr int kFwdTMaxSteps = 8000;

inline constexpr double kRefDsApproach = 0.38;
inline constexpr double kFwdASpeed = 1.6;
inline constexpr double kFwdALookahead = 1.8;

inline constexpr int kReverseBezierSamples = 48;
inline constexpr double kRefDsReverse = 0.22;
inline constexpr double kRevTargetSpeed = 0.75;
inline constexpr double kRevLookahead = 1.15;
inline constexpr double kRevStopTol = 0.55;
inline constexpr double kRevYawTol = 0.32;

inline constexpr double kPolylineTangentEndOffsetM = 0.02;

}  // namespace autopark::sim_tune
