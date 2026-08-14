# SAUVC-RPI —— Orca AUV 載具控制堆疊

> **名稱說明**：repo 名還叫 `SAUVC-RPI`，但樹莓派已經退場。這裡是載具的
> **控制堆疊**（vehicle control stack）：接收目標、跑 PID、把力分配給推進器。
> 它跑在哪塊板子上是部署細節 —— 目前與感知決策堆疊（`SAUVC-JETSON`）
> 一起跑在同一塊 Jetson Orin NX 上，各自在獨立 container 裡。
> 改名會與 super-repo 的建立一併進行。

## 這個 repo 負責什麼

```text
   SAUVC-JETSON  (separate container)
   perception -> BehaviorTree decision
            |
            |  control/wrench_sources/decision  (Wrench, 50 Hz)
            |  control/targets/depth_m          (Float64)
            v
+-----------------------------------------------------------------------+
| this repo                                                             |
|                                                                       |
|  STM32 ------> sensors ------> state/depth_m ------> depth PID        |
|  (pressure/IMU)                                          |            |
|                                                          v            |
|  GUI manual -----------------------> control/wrench_sources/*         |
|  autonomy decision ----------------->        |                        |
|                                              v                        |
|                                         wrench_sum                    |
|                                              |  control/wrench_command|
|                                              v                        |
|                   thrust allocation (pseudo-inverse + clamp)          |
|                                              |                        |
|                        +---------------------+---------------------+  |
|                        v                                           v  |
|               force -> PWM -> STM32                     ros_gz_bridge |
|                   (hardware)                              (simulation)|
+-----------------------------------------------------------------------+
```

核心設計是 **wrench 匯流排**：所有「想讓載具動」的來源（深度 PID、GUI 手動、
Autonomy 決策）都只做一件事 —— 發布一個 `geometry_msgs/Wrench` 到自己專屬的
`control/wrench_sources/*`。新增控制行為只要多發一個 topic 並在設定裡加一行，
不用碰任何下游程式碼。細節見 [docs/ARCHITECTURE.html](docs/ARCHITECTURE.html)。

## 取得

**平常不用直接開這個 repo。** 從 [super-repo](https://github.com/NCTU-AUV/SAUVC)
一個指令就會把這個堆疊連同感知決策與模擬一起拉起來：

```shell
cd ../          # SAUVC super-repo
make up && make build && make launch
```

底下是單獨開發本 repo 時用的流程。

```shell
git clone https://github.com/NCTU-AUV/SAUVC-RPI.git
cd SAUVC-RPI
git submodule update --init --recursive
```

## 快速開始

```shell
make init      # 建容器 + colcon build（第一次，或改過原始碼後）
make launch    # 實機啟動
```

GUI：<http://localhost>（從別台機器則是 `http://<載具 IP>`）

GUI 的 Mission 面板會顯示 autonomy 堆疊行為樹的即時狀態。資料來自
`/orca/decision/status_json`——那是 `DecisionStatus` 的 JSON 鏡像，因為
`orca_interface` 沒有 build 進這個容器，原本的訊息型別在這裡反序列化不了。
面板顯示 `offline` 就是那條 topic 沒進來（autonomy 容器沒起來，或還在
重建 TensorRT engine），不是任務閒置。

### 模擬

需要另外跑起 [SAUVC-Simulation](https://github.com/NCTU-AUV/SAUVC-Simulation)
的 Gazebo 場景，然後：

```shell
make sim_launch     # 啟動控制堆疊（模擬模式）
make sim_status     # 節點 / topic / lifecycle 狀態
make stop           # 停掉整個堆疊
```

實機與模擬共用同一個 launch 檔，差別只有 `sim:=true`：

```shell
ros2 launch orca_bringup bringup.launch.py            # 實機
ros2 launch orca_bringup bringup.launch.py sim:=true  # 模擬
```

`sim:=true` 會跳過硬體專屬節點（micro-ROS agent、STM32 燒錄、ESC 初始化、
力→PWM 轉換），並疊上 `sim_overrides.yaml` 放寬安全門檻。其餘節點與參數
完全相同 —— 這是刻意的，模擬與實機各一份 launch 檔必然會漂移。

## 設定放在哪

只有兩個地方，沒有第三個：

| 檔案 | 內容 | 什麼時候會動 |
|---|---|---|
| [`.env`](.env) | ROS/DDS 網路、namespace、裝置路徑、bag 位置 | 換機器、換序列埠 |
| [`rpi_ros2_ws/src/orca_bringup/config/`](rpi_ros2_ws/src/orca_bringup/config/) | 控制參數、推進器幾何、bag 錄製清單 | 池邊調參、換零件 |

`config/` 底下：

- `orca_params.yaml` —— PID 增益、深度偏壓、逾時。**池邊調參動這份。**
- `hardware.yaml` —— 推進器幾何、出力上限、ESC 時序。換零件才動。
- `sim_overrides.yaml` —— 模擬專用疊加值。刻意保持很短。
- `record_topics.yaml` —— bag 錄製清單。

不要把數值寫回 launch 檔或節點預設值 —— 那正是重構前的狀態，
同一個參數散在三個地方而且互相矛盾。

### 執行期調參

PID 增益每個控制迴圈重新讀取，所以 `ros2 param set` 立即生效，不用重啟：

```shell
ros2 param set /orca_auv/depth_pid_controller_node proportional_gain 45.0
```

調完把成果存下來，不要靠手抄：

```shell
make dump_params    # → snapshots/params.yaml
```

## 操作

| 指令 | 用途 |
|---|---|
| `make launch` / `make sim_launch` | 啟動（實機／模擬） |
| `make launch_detached` + `make launch_logs` | 背景啟動並跟 log |
| `make stop` | 停掉整個堆疊（含孤兒程序） |
| `make stack_status` | 現在有幾個節點在跑 |
| `make sim_status` / `make sim_check` | 節點、topic、lifecycle、資料流檢查 |
| `make compose_shell` | 進容器 |
| `make dump_params` / `make snapshot` | 參數與節點快照 |
| `make bag_list` / `make bag_info` | 看錄了哪些 bag |

### 控制模式

由 `supervisor_node` 集中管理，PID 節點本身完全不知道 kill switch 或
逾時的存在，只回應標準的 lifecycle 轉換。

```shell
ros2 service call /orca_auv/system_manager/set_mode/depth_hold  std_srvs/srv/Trigger {}
ros2 service call /orca_auv/system_manager/set_mode/autonomous  std_srvs/srv/Trigger {}
ros2 service call /orca_auv/system_manager/set_mode/safe_disabled std_srvs/srv/Trigger {}
```

`depth_hold` 與 `autonomous` 可以疊加（模式會變成 `AUTONOMOUS_AND_DEPTH_HOLD`）。
任何一個安全前提被打破 —— kill switch 觸發、深度感測器逾時、
Autonomy 的 decision wrench 逾時 —— 都會立刻進 `FAULT` 並停掉所有輸出。

**FAULT 是鎖存狀態。** 在 FAULT 中呼叫 `depth_hold` 或 `autonomous` 會被回絕
（`success=False`，訊息帶著故障原因），控制器狀態完全不會被改動。
要恢復必須先明確地經 `safe_disabled` 或 `manual` 清除 FAULT，這是唯一的出口。

## Bag 錄製

隨啟動自動開始錄（比賽時沒有人會記得按錄影），要關就 `record:=false`。
用 mcap 格式、每 120 秒分段、啟動前檢查磁碟空間。bag 落在 host 的 `bags/`。

> **AUV 是靠 kill switch 直接斷電關機的**，所以 bag 目錄通常不會有
> `metadata.yaml`，`ros2 bag info` 會打不開。這是正常的，資料沒有遺失。
> 撈資料前先跑一次：
>
> ```shell
> ros2 bag reindex <bag_dir> -s mcap
> ```

## Workspace 結構

```text
rpi_ros2_ws/src/
├── orca_bringup/     # 所有 launch 與設定的唯一來源
├── sensors/          # STM32 原始訊號 → 控制層可用的回饋
├── control/          # 通用 PID（含積分抗飽和與輸出限幅）
├── depth_control/    # 深度力 → wrench 匯流排的一路來源
├── wrench_sum/       # 匯流排加總（每來源獨立 timeout）
├── thrusters/        # 6-DOF → 8 顆推進器分配、飽和限幅、力→PWM
├── system_manager/   # 控制模式狀態機 + 安全門檻
├── gui/              # ROS 2 ↔ WebSocket 橋接 + 網頁操作介面
└── legacy/           # 不編譯、不啟動，原始碼保留供查閱
```

`legacy/` 裡是退場的底部相機光流視覺伺服鏈，理由見
[legacy/README.md](rpi_ros2_ws/src/legacy/README.md)。

## 相關文件

- [docs/ARCHITECTURE.html](docs/ARCHITECTURE.html) —— 系統怎麼運作
- [../docs/HANDOFF.md](../docs/HANDOFF.md) —— 座標慣例、已知缺陷、驗收方式
- [../README.md](../README.md) —— super-repo：一次啟動整套系統
- [../docs/REFACTOR_PLAN.md](../docs/REFACTOR_PLAN.md) —— 重構計畫與決策紀錄
- [../docs/SIMULATION_FINDINGS.md](../docs/SIMULATION_FINDINGS.md) —— 三容器全鏈路實測報告
