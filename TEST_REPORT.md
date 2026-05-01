# autopark_cpp 測試流程報告

本文檔描述 `autopark_cpp` 專案的**建置驗證**、**功能測試流程**、**通過標準**、**模組對應關係**與**與 Python 版差異**，供回歸與交付檢查使用。

---

## 1. 文件目的與範圍

| 項目 | 說明 |
|------|------|
| 測試對象 | `autopark_run` 可執行檔（C++ 規劃入口） |
| 涵蓋內容 | CMake 建置、CLI 參數、JSON 輸入/輸出、核心規劃管線（幾何/換道/overshoot/倒車 Bezier） |
| 不涵蓋 | Matplotlib 繪圖、與 Python 版 `run_parking.py` 完全一致的運動學模擬與碰撞檢查 |

---

## 2. 前置條件

| 項目 | 說明 |
|------|------|
| 編譯器 | 支援 **C++17** 的編譯器（如 MSVC、GCC、Clang） |
| CMake | **3.20** 或以上 |
| 網路 | **不需要**：`nlohmann/json` 已置於 `third party/nlohmann/` |
| 場景檔 | 預設使用 **`autopark_cpp/data/parking_scene.json`**（執行時工作目錄應為 `autopark_cpp/`，相對路徑 `data/parking_scene.json`） |

---

## 3. 建置測試流程（必做）

### 3.1 步驟

1. 開啟終端，進入 **`autopark_cpp` 目錄**（本 C++ 子專案根目錄）：
   ```powershell
   cd <你的路徑>\autopark_cpp
   ```
2. 產生建置目錄並設定：
   ```powershell
   cmake -S . -B build
   ```
3. 編譯 Release（或預設組態）：
   ```powershell
   cmake --build build --config Release
   ```

### 3.2 通過標準

- CMake **Configure / Generate** 無錯誤。
- 編譯 **0 errors**。
- 產物存在（依產生器與平台而定，常見為）：
  - `autopark_cpp\build\autopark_run.exe`（Ninja / MinGW 等）
  - 或 `autopark_cpp\build\Release\autopark_run.exe`（部分 Visual Studio 多組態）

### 3.3 失敗時檢查

- 找不到編譯器：確認 PATH 或於 IDE 開發者命令提示字元執行。
- 找不到 `nlohmann/json.hpp`：確認 `third party/nlohmann/json.hpp` 存在且未被刪除。

---

## 4. 執行與功能測試流程

以下假設當前目錄為 `autopark_cpp`，且可執行檔為 `.\build\autopark_run.exe`（若路徑不同請替換為實際產物路徑）。

### 4.1 測試 T1：基本執行（使用場景內 `target_slots`）

**目標**：驗證讀取場景、規劃、寫出 `parking_result` 風格 JSON。

**命令**：

```powershell
.\build\autopark_run.exe --map data\parking_scene.json --out outputs\parking_result_cpp.json
```

**通過標準**：

- 規劃成功時進程退出碼為 **0**；規劃失敗（例如 `message` 非成功）時為 **1**。
- 終端最後一行出現：`[ok] wrote <path>`（路徑以實際 `--out` 為準）。
- 輸出檔為合法 JSON，且至少包含：
  - `target`、`ego_start`、`slot_yaw`、`vehicle`
  - `approach_near`、`staging`、`slot_bounds`
  - `forward_polyline`、`reverse_polyline`（陣列）
  - `success`、`sim_success`（C++ 版目前與規劃成功對齊）
  - `message`、`sim_message`
  - `sim_forward`、`sim_reverse`（目前可為空陣列，見第 6 節）
- `viz_densify_step_m`、`viz_body_sample_stride`（預設 0.45 m、50）
- `forward_body_samples`、`reverse_body_samples`：沿密化路徑每 50 點一筆 `{x,y,yaw}`（參考軌跡車身姿態，非運動學模擬步）

### 4.2 測試 T2：`--plan-row0` 覆寫目標

**目標**：驗證忽略 JSON 內 `target_slots`，改為 **row 0 第一個空位**。

**命令**：

```powershell
.\build\autopark_run.exe --map data\parking_scene.json --out outputs\parking_result_cpp.json --plan-row0
```

**通過標準**：

- 輸出 JSON 中 `target.row` 為 **0**。
- `target.index` 為 **row 0 中未被 `occupied_slots` 佔用的最小 index**（依目前場景資料驗證一次即可）。

### 4.3 測試 T3：參數覆寫（overshoot / near / max steer）

**目標**：驗證 CLI 參數傳入並反映於輸出與訊息字串。

**命令範例**：

```powershell
.\build\autopark_run.exe --map data\parking_scene.json --out outputs\parking_result_cpp.json --overshoot-m 6.0 --near-m 3.0 --max-steer-deg 30
```

**通過標準**：

- 輸出 JSON 中 `overshoot_m` 為 **6.0**。
- `max_steer_deg` 為 **30**。
- `message` 字串包含 `|steer|<=30 deg`。

### 4.4 測試 T4：軌跡可視化（SVG）

**目標**：驗證與 Python `--plot-out` 類似行為：寫入軌跡圖檔；可選開啟預設檢視器。

**命令**：

```powershell
.\build\autopark_run.exe --map data\parking_scene.json --out outputs\parking_result_cpp.json --plot-out outputs\parking_plot_cpp.svg
```

**通過標準**：

- 終端出現：`[ok] wrote plot <path>`。
- 產生之檔案為 **SVG**（可用瀏覽器或 Edge 開啟），內含地圖邊框、道路、車位、前進/倒車折線。
- 選項 `--no-plot` 時**不**產生 SVG；`--show-plot` 在寫檔後嘗試以系統預設程式開啟（失敗時可能印出 `[warn]`，不強制退出碼失敗）。

### 4.5 測試 T5：說明與錯誤路徑

| 步驟 | 命令 | 預期 |
|------|------|------|
| Help | `.\build\autopark_run.exe --help` | 印出選項說明（含 `--plot-out` / `--no-plot` / `--show-plot`），退出碼 **0** |
| 場景檔不存在 | `.\build\autopark_run.exe --map data\nonexistent.json --out outputs\x.json` | stderr 含 `scene file not found`，退出碼 **1** |
| JSON 解析失敗 | 使用非法 JSON 檔作為 `--map` | stderr 含 `failed to parse json`，退出碼 **1** |
| 無法寫入輸出 | `--out` 指向無寫入權限路徑 | stderr 含 `cannot write output file`，退出碼 **1** |

---

## 5. 模組與測試對應

| 測試重點 | 對應原始碼 | 驗證方式 |
|----------|------------|----------|
| 車位矩形、車道群組 | `geometry_lane.cpp` | T1/T2 檢查 `slot_bounds`、`slot_yaw`、換路徑是否合理 |
| 上下主道 connector | `connector_transfer.cpp` | T1：ego 與目標不同層時 `forward_polyline` 含多段折線 |
| near + overshoot（含方向修正） | `forward_overshoot.cpp` | T3：改 `overshoot-m` 後 `staging` 與折線長度變化 |
| 倒車 Bezier 參考線 | `reverse_bezier.cpp` | T1：`reverse_polyline` 非空且終點接近目標中心附近 |
| 調度與 JSON 輸出 | `main.cpp` | 全部測試；檢查欄位完整性與退出碼 |
| 軌跡 SVG 匯出 | `viz_svg.cpp` | T4：檔案存在且可開啟 |

---

## 6. 與 Python 版 `run_parking.py` 的差異（測試時須知）

| 項目 | C++ 版 | Python 版 |
|------|--------|-------------|
| 可視化 | **SVG**（`viz_svg.cpp`，無第三方繪圖庫） | Matplotlib PNG（`plot_parking.py`） |
| 前進/倒車運動學模擬 | 未實作（`sim_forward`/`sim_reverse` 為空） | 有 Pure Pursuit + 自行車模型 |
| 碰撞與可通行域檢查 | 未完整移植 | 有 |
| `sim_success` 語意 | 目前與規劃 `success` 對齊 | 獨立模擬結果 |

因此：**C++ 輸出 JSON 欄位結構可對齊，但數值與 Python 版不必逐點一致**；若需對齊，須在 C++ 中補齊模擬模組後再做數值比對測試。

---

## 7. 建議回歸檢查清單（簡表）

- [ ] 3.1 建置成功  
- [ ] T1 基本執行成功且 JSON 欄位齊全  
- [ ] T2 `--plan-row0` 目標正確  
- [ ] T3 參數覆寫反映於輸出  
- [ ] T4 SVG 軌跡圖產生成功  
- [ ] T5 錯誤路徑退出碼與訊息符合預期  

---

## 8. 文件版本

- 路徑：`autopark_cpp/TEST_REPORT.md`
- 與 `README.md`、`CMakeLists.txt` 及 `src/*.cpp` 結構同步維護
