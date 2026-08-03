FROM ros:humble

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive

# -----------------------------------------------------------------------------
# micro-ROS agent（與 STM32 韌體的 serial 橋樑）
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends git \
 && rm -rf /var/lib/apt/lists/*

# 注意：這一層一定要自己先 apt-get update。
# 上一層結尾清掉了 /var/lib/apt/lists，而 rosdep install 內部是直接呼叫
# `apt-get install`，沒有套件清單就會以 "Unable to locate package" 失敗。
# 每個會裝東西的 RUN 都必須自帶 update，不要依賴前一層留下的快取。
RUN source /opt/ros/humble/setup.bash && \
    apt-get update && \
    cd ~/ && \
    mkdir uros_ws && cd uros_ws && \
    git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup && \
    rosdep update && rosdep install --from-paths src --ignore-src -y && \
    colcon build && \
    source install/local_setup.bash && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh

RUN echo "source ~/uros_ws/install/local_setup.bash" >> /etc/bash.bashrc

# -----------------------------------------------------------------------------
# 韌體燒錄與飛控介面
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        stlink-tools \
        ros-humble-mavros \
 && rm -rf /var/lib/apt/lists/*

RUN source /opt/ros/humble/setup.bash && ros2 run mavros install_geographiclib_datasets.sh

# -----------------------------------------------------------------------------
# Workspace 執行期相依
#
# 這些以前是靠 `make init` 在執行中的容器裡跑 `rosdep install` 補上的，
# 因此只要 container 被 recreate（compose down / up、換機器）就會消失，
# 造成「映像 build 成功但系統跑不起來」。一律改為裝進映像。
# 新增 package 相依時請同步更新這裡，不要只依賴 rosdep。
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-aiohttp \
        python3-numpy \
        python3-opencv \
        ros-humble-cv-bridge \
        ros-humble-vision-opencv \
        ros-humble-web-video-server \
 && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# Bag 錄製
#
# 用 mcap 而非預設的 sqlite3：AUV 是靠 kill switch 直接斷電關機的，
# sqlite3 遇到硬斷電容易整包損毀，mcap 對截斷友善，最壞只損失尾端。
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rosbag2-storage-mcap \
 && rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# 除錯工具（可選，只在有 GUI 的開發機上用得到）
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-rqt-image-view \
        ros-humble-rqt-graph \
        ros-humble-rqt-topic \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /root

CMD ["/bin/bash"]
