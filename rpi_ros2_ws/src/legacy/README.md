# legacy/

這個目錄裡的 package **不會被編譯、不會被啟動**（同層的空檔案 `COLCON_IGNORE`
會讓 colcon 跳過整個子樹）。原始碼保留在 git 裡供查閱與日後取回。

## 為什麼移到這裡

底部相機的 LK 光流視覺伺服鏈在 2026-08 的重構中退場，原因不是它壞了，而是：

1. **硬體退場。** RPi 移除、STM32 直接接上 Jetson 之後，底部相機由 Autonomy 側的
   `camera_selector` 統一處理（它本來就有自己的 USB 底部相機輸入路徑），本 repo
   這條鏈完全重複。
2. **主線改變。** 現在的主線是「接收 Autonomy 的 `wrench_sources/decision` 與
   `targets/depth_m`，用 PID 達成目標」。x/y/yaw 的視覺定位不在這條路徑上。

## 內容與相依關係

| Package | 內容 |
|---|---|
| `xy_control` | `lk_total_transform_node`（LK 光流視覺里程計，1,396 行 / 約 70 個參數）、`bottom_camera_pid_bridge_node`、`yaw_reference_unwrapper_node`、`waypoint_target_publisher`（`MoveToPoint` action server）、`dive_then_forward_mission_node`（1,093 行的顯式任務狀態機） |
| `bottom_camera` | V4L2 相機驅動節點 |
| `xy_translation_control_interfaces` | 只定義 `MoveToPoint.action` |

`lk_total_transform_node` 是以下四個 topic 的唯一發布者，因此整條鏈是一起進退的：

- `camera/bottom/pose_px`
- `control/pid/bottom_camera/x/feedback_px`
- `control/pid/bottom_camera/y/feedback_px`
- `state/bottom_camera/yaw_rad`

`dive_then_forward_mission_node` 一併移入：它依賴 `MoveToPoint` action 與
supervisor 的 `bottom_camera_hold` 模式，且其任務邏輯已被 Autonomy 側的
BehaviorTree（`orca_decision`）取代。

## 要取回的話

1. 刪掉 `src/legacy/COLCON_IGNORE`
2. 把需要的 package 移回 `src/`
3. 還原 `system_manager`（`bottom_camera_pid_fbc` 群組、`BOTTOM_CAMERA_HOLD` /
   `DEPTH_AND_BOTTOM_CAMERA_HOLD` 兩個模式、`SafetyMonitor.bottom_camera_ready()`）
   與 `gui`（bottom-camera 面板與相關 protocol 常數）

移除當下的完整脈絡見 [docs/REFACTOR_PLAN.md](../../../docs/REFACTOR_PLAN.md) §3.1。
