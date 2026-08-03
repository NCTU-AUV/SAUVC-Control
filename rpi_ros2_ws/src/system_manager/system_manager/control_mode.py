from enum import Enum


class ControlMode(Enum):
    """載具的高層控制模式。

    這是獨立於各節點 lifecycle 狀態之外的一層抽象：lifecycle 描述「某個控制器
    現在有沒有在運作」，ControlMode 描述「操作者要載具處於哪種行為」。
    兩者的對應關係由 supervisor_node 維護。

    AUTONOMOUS 代表放行 Autonomy 堆疊（SAUVC-JETSON）的
    control/wrench_sources/decision。在有這個模式之前，要讓決策層的指令
    到得了推進器只能先進 MANUAL（語意矛盾，而且會關掉深度 PID）或 DEPTH_HOLD
    （順帶啟用深度 PID），且沒有任何機制確認決策來源還活著。

    BOTTOM_CAMERA_HOLD 與 DEPTH_AND_BOTTOM_CAMERA_HOLD 已隨底部相機光流鏈
    一併移除（見 src/legacy/README.md）。
    """

    SAFE_DISABLED = "SAFE_DISABLED"
    MANUAL = "MANUAL"
    DEPTH_HOLD = "DEPTH_HOLD"
    AUTONOMOUS = "AUTONOMOUS"
    AUTONOMOUS_AND_DEPTH_HOLD = "AUTONOMOUS_AND_DEPTH_HOLD"
    FAULT = "FAULT"
