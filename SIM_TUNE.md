# 模擬參數調試說明

**所有可調數值已集中在** `include/autopark/sim_tune.hpp`（與 `ap::sim_tune` 相同；標頭內以 `namespace autopark::sim_tune` 定義）。修改後重新編譯 `autopark_run` 即可。

Python 對照（語意一致）在 `scripts/run_parking.py` 頂部同名常數（如 `REVERSE_PHASE_D_LATERAL_TOL_M` 等）。

## 為什麼常是 row1 / row2 失敗、看起來在「擺動」？

- **幾何**：row1、row2 是**中間排**車位，左右/上下鄰近格與車道縫隙較「擠」，倒車稍偏就容易觸發 `reverse sim collision`；邊界感比最上排 row0、最下排 row3 更強。
- **主道分組**：`geometry_lane.cpp` 裡 `row<=1` 走 **upper** 主道（`LaneYForRow`=37.5），`row>=2` 走 **lower**（12.0）。row1 與 row2 分屬兩條主道，**前進模擬結束時**車輛姿態、進入倒車貝茲的切向與目標車位 `SlotYaw` 組合不同，**Pure Pursuit** 若與參考線切向差大，會在短前瞻下反覆左右打舵 → 疊成擺動。
- **控制器**：非 row 專用 bug，而是 **row1/2 場景下誤差更常落入「敏感區」**。

已為倒車單獨加入較低 **`kMaxSteerRateRadSReverse`**、較大前瞻下限、**`kReversePpSteerGainScale`** 等，專門壓低擺動（見 `sim_tune.hpp`）。

## 倒車抖動 / 對不齊終點時建議優先順序

| 參數 | 預設 | 調整方向 |
|------|------|----------|
| `kReverseLookaheadFloorM` | （見 hpp） | **略增** → 降低 PP 舵角增益，減少左右晃 |
| `kNearGoalLdEffMinReverse` | （見 hpp） | **略增** → 近距離倒車仍保持較大前瞻 |
| `kReversePpSteerGainScale` | （見 hpp） | **略減**（如 0.82～0.88）→ 直接削弱 PP 舵指令幅度 |
| `kMaxSteerRateRadSReverse` | （見 hpp） | **略減** → 舵角變化更慢、軌跡更柔 |
| `kNearGoalLdEffMin` | 0.32 | **略增**（如 0.40～0.50）→ 接近終點時前瞻不會過短 |
| `kMaxSteerRateRadS` | 1.2 | **略降**（如 0.9～1.0）→ 舵角變化更平滑 |
| `kRevTargetSpeed` | 0.75 | **略降**（如 0.55～0.65）→ 倒車更慢，易穩定 |
| `kRevLookahead` | 1.15 | **略增**（如 1.3～1.6）→ 追蹤更「平滑」、轉向更緩 |
| `kRefDsReverse` | 0.22 | **略降**（如 0.16～0.18）→ 參考點更密，前瞻索引較不跳 |
| `kReversePhaseDLateralTolM` / `kReversePhaseDYawTolRad` | 3.0 / 0.03 | **略放寬**→ 較早進入 Phase-D 直線尾段；**略收緊**→ 較晚進入，避免早切 |

## Phase-D 滯回（`bicycle_sim.cpp` 使用）

| 參數 | 預設 | 說明 |
|------|------|------|
| `kReversePhaseDExitYawFactor` | 2.2 | 退出直線模式時航向誤差需超過「入帶」閾值的倍數 |
| `kReversePhaseDExitLatFactor` | 1.45 | 同上，橫向誤差倍數 |

**略增大**倍數 → 更難退出直線段，減少 PP↔0 舵切換；**略減小** → 較早回到 PP 跟線。

## 其他檔案

- `src/bicycle_sim.cpp`：僅引用 `sim_tune.hpp`，並含近距離縮放區間（`kNearGoalShrinkDistM` 等）。
- `src/parking_simulate.cpp`：`using namespace sim_tune` 後使用 `kFwd*`、`kRev*` 等。
- `src/main.cpp`：`wheelbase_m` 輸出使用 `sim_tune::kWheelbaseM`。
