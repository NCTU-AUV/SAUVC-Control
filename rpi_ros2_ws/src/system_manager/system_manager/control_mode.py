from enum import Enum


class ControlMode(Enum):
    """載具的高層控制模式。

    這是獨立於各節點 lifecycle 狀態之外的一層抽象：lifecycle 描述「某個控制器
    現在有沒有在運作」，ControlMode 描述「操作者要載具處於哪種行為」。
    兩者的對應關係由 supervisor_node 維護。

    BOTTOM_CAMERA_HOLD 與 DEPTH_AND_BOTTOM_CAMERA_HOLD 已隨底部相機光流鏈
    一併移除（見 src/legacy/README.md）。
    """

    SAFE_DISABLED = "SAFE_DISABLED"
    MANUAL = "MANUAL"
    DEPTH_HOLD = "DEPTH_HOLD"
    FAULT = "FAULT"
