# -----------------------------------------------------------------------------
# 環境設定的唯一來源是 .env（docker compose 也讀同一份）。
# 這裡 include 進來是為了讓 `docker compose exec` 出來的 shell 拿到同樣的值。
# 不要在本檔案裡另外寫死 ROS_DOMAIN_ID / RMW_IMPLEMENTATION 等值。
# -----------------------------------------------------------------------------
ifneq (,$(wildcard .env))
include .env
export
endif

# Define variables
IMAGE_NAME ?= orca-auv-rpi-ros2-image
CONTAINER_NAME ?= orca-auv-rpi-ros2-container
WORKSPACE := rpi_ros2_ws
IMAGE_OWNER_NAME ?= dianyueguo
PWD := $(shell pwd)
# Prefer Docker Compose v2 (docker compose) but fall back to v1 (docker-compose); allow override via env/CLI
COMPOSE ?= $(shell \
	if docker compose version >/dev/null 2>&1; then \
		printf "docker compose"; \
	elif docker-compose --version >/dev/null 2>&1; then \
		printf "docker-compose"; \
	else \
		printf ""; \
	fi)
ifeq ($(strip $(COMPOSE)),)
$(error Docker Compose not found: install Docker Compose v2 (docker compose) or v1 (docker-compose), or set COMPOSE to your compose binary)
endif
# Host display/X11 wiring varies by platform; set defaults and skip mounts when missing
ifeq ($(OS),Windows_NT)
	HOST_DISPLAY ?= host.docker.internal:0.0
	XAUTH_FILE :=
else
	UNAME_S := $(shell uname -s)
	ifeq ($(UNAME_S),Darwin)
		HOST_DISPLAY ?= host.docker.internal:0
	else
		HOST_DISPLAY ?= $(DISPLAY)
	endif
	XAUTH_FILE ?= $(HOME)/.Xauthority
endif
XAUTHORITY ?= /tmp/.Xauthority
XAUTH_FLAGS := $(if $(and $(XAUTH_FILE),$(wildcard $(XAUTH_FILE))),-v $(XAUTH_FILE):/tmp/.Xauthority:ro -e XAUTHORITY=/tmp/.Xauthority,)
ROS_DOMAIN_ID ?= 0
ROS_LOCALHOST_ONLY ?= 0
RMW_IMPLEMENTATION ?= rmw_fastrtps_cpp
# 見 .env 的說明：這是正確性設定，不是效能調校。
FASTDDS_BUILTIN_TRANSPORTS ?= UDPv4
ORCA_NAMESPACE ?= orca_auv
ROS_NET_ENV := ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) ROS_LOCALHOST_ONLY=$(ROS_LOCALHOST_ONLY) RMW_IMPLEMENTATION=$(RMW_IMPLEMENTATION) FASTDDS_BUILTIN_TRANSPORTS=$(FASTDDS_BUILTIN_TRANSPORTS) ORCA_NAMESPACE=$(ORCA_NAMESPACE) ORCA_STM32_PORT=$(ORCA_STM32_PORT)
BRINGUP_LOG ?= /tmp/orca_bringup.log
SNAPSHOT_DIR ?= snapshots

# -----------------------------------------------------------------------------
# 停掉整個堆疊。
#
# 用單一 pattern `rpi_ros2_ws/install` 涵蓋所有節點，而不是逐一列出節點名 ——
# 舊版那份 17 行的 pkill 清單每次新增／改名節點都會漏掉，漏掉的節點會變成
# 孤兒程序繼續跑，下次啟動就變成同時有兩套 supervisor 在互相覆蓋狀態
# （實測症狀：set_mode 回 success，但 system_manager/mode 一直廣播 SAFE_DISABLED）。
#
# `[r]` 括號技巧是必要的：pkill -f 會匹配整條命令列，而執行這段的
# `bash -lc '...'` 自己的命令列就含有這個 pattern，不繞開的話會先殺掉自己。
# -----------------------------------------------------------------------------
STOP_STACK := \
	pkill -INT -f '[r]os2 launch' || true; \
	sleep 2; \
	pkill -9 -f '[r]pi_ros2_ws/install' || true; \
	pkill -9 -f '[w]eb_video_server' || true; \
	pkill -9 -f '[m]icro_ros_agent' || true; \
	sleep 1

.PHONY: all compose_up compose_start compose_down compose_build compose_shell init launch launch_detached launch_logs compose_init compose_launch compose_launch_detached compose_clean clean update_image

all: init launch

update_image:
	docker buildx build --pull --platform=linux/arm64,linux/amd64 -t $(IMAGE_OWNER_NAME)/$(IMAGE_NAME):latest . --push

compose_up:
	@echo "Starting compose stack"
	@mkdir -p $(dir $(XAUTH_FILE))
	@touch $(XAUTH_FILE)
	HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) build --pull
	HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) up -d --no-build

compose_down:
	$(COMPOSE) down

compose_build:
	HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) build --pull

compose_start:
	@echo "Starting compose stack without rebuilding"
	HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) up -d --no-build

compose_shell:
	HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec orca /bin/bash -lc "\
		source /opt/ros/humble/setup.bash; \
		if [ -f /root/uros_ws/install/local_setup.bash ]; then \
			source /root/uros_ws/install/local_setup.bash; \
		fi; \
		if [ -f $(WORKSPACE)/install/setup.bash ]; then \
			source $(WORKSPACE)/install/setup.bash; \
		fi; \
		exec bash"

init: compose_up
	HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec orca /bin/bash -lc "\
		source /opt/ros/humble/setup.bash && \
		cd $(WORKSPACE) && \
		rosdep install --from-paths src --ignore-src -y && \
		colcon build --symlink-install && \
		echo \"source $(WORKSPACE)/install/setup.bash\" >> /etc/bash.bashrc"

launch: compose_up
	HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec orca /bin/bash -lc "\
		cd $(WORKSPACE) && \
		source /opt/ros/humble/setup.bash && \
		source /root/uros_ws/install/local_setup.bash && \
		source install/setup.bash && \
		ros2 launch orca_bringup bringup.launch.py"

launch_detached: compose_up
	@echo "Launching ROS stack in detached mode"
	HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec -T -d orca /bin/bash -lc "\
		cd $(WORKSPACE) && \
		source /opt/ros/humble/setup.bash && \
		source /root/uros_ws/install/local_setup.bash && \
		source install/setup.bash && \
		rm -f $(BRINGUP_LOG); \
		echo \"Starting orca_bringup at \$$(date -Is)\" > $(BRINGUP_LOG); \
		exec ros2 launch orca_bringup bringup.launch.py >> $(BRINGUP_LOG) 2>&1"

launch_logs: compose_start
	@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec -T orca /bin/bash -lc "\
		tail -n 200 -f $(BRINGUP_LOG)"

compose_init: init

compose_launch: launch

compose_launch_detached: launch_detached

compose_clean:
	$(COMPOSE) down -v


clean: compose_clean
	-@docker rmi $(IMAGE_OWNER_NAME)/$(IMAGE_NAME):latest
	rm -rf $(WORKSPACE)/build $(WORKSPACE)/install $(WORKSPACE)/log


# --------------------------------------------------------------------
# Simulation targets
# --------------------------------------------------------------------
# These targets are for controlling SAUVC-Simulation from SAUVC-RPI.
# They intentionally DO NOT start hardware-only nodes:
# - stm32_flasher_node
# - micro_ros_agent
# - thruster PWM conversion node
#
# Expected data flow:
# GUI -> control/wrench_sources/gui
#     -> wrench_sum_node
#     -> control/wrench_command
#     -> wrench_to_individual_thrusters_output_forces_node
#     -> thrusters/thruster_0~7/force_N
#     -> SAUVC-Simulation ros_gz_bridge

.PHONY: \
	sim_launch sim_launch_detached sim_stop sim_status sim_check sim_logs \
	sim_gui_detached sim_wrench_sum_detached sim_activate_wrench_sum \
	sim_set_manual sim_thruster_allocator_detached

ROS_SETUP := cd $(WORKSPACE) && \
	source /opt/ros/humble/setup.bash && \
	if [ -f /root/uros_ws/install/local_setup.bash ]; then \
		source /root/uros_ws/install/local_setup.bash; \
	fi && \
	source install/setup.bash &&

sim_launch: sim_launch_detached sim_status
	@echo ""
	@echo "Simulation control stack started."
	@echo "Open GUI at: http://localhost/controller"
	@echo "Or from another device: http://<HOST_IP>/controller"

sim_launch_detached: compose_up
	@echo "Stopping old SAUVC-RPI simulation-control nodes..."
	-@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec orca /bin/bash -lc "\
		$(STOP_STACK); \
		source /opt/ros/humble/setup.bash; \
		ros2 daemon stop || true"
	@echo "Starting simulation launch: GUI, tile-line tracking, supervisor, controllers, wrench sum, thruster force allocator..."
	@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec -d orca /bin/bash -lc "\
		$(ROS_SETUP) \
		exec ros2 launch orca_bringup bringup.launch.py sim:=true \
			> /tmp/sauvc_rpi_sim_control.log 2>&1"
	@echo "Waiting for lifecycle services..."
	@sleep 4
	@$(MAKE) sim_activate_wrench_sum
	@echo "Done."

sim_set_manual:
	@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec orca /bin/bash -lc "\
		$(ROS_SETUP) \
		for i in \$$(seq 1 30); do \
			ros2 service list | grep -q '/orca_auv/system_manager/set_mode/manual' && break; \
			sleep 0.2; \
		done; \
		timeout 20s ros2 service call /orca_auv/system_manager/set_mode/manual std_srvs/srv/Trigger '{}' || true"

sim_activate_wrench_sum:
	@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec orca /bin/bash -lc "\
		$(ROS_SETUP) \
		for i in \$$(seq 1 20); do \
			ros2 node list | grep -q '/orca_auv/wrench_sum_node' && break; \
			sleep 0.2; \
		done; \
		state=\$$(timeout 10s ros2 lifecycle get /orca_auv/wrench_sum_node 2>/dev/null | awk '{print \$$1}'); \
		if [ \"\$$state\" = \"unconfigured\" ]; then \
			timeout 15s ros2 lifecycle set /orca_auv/wrench_sum_node configure || true; \
		fi; \
		state=\$$(timeout 10s ros2 lifecycle get /orca_auv/wrench_sum_node 2>/dev/null | awk '{print \$$1}'); \
		if [ \"\$$state\" = \"inactive\" ]; then \
			timeout 15s ros2 lifecycle set /orca_auv/wrench_sum_node activate || true; \
		fi; \
		timeout 10s ros2 lifecycle get /orca_auv/wrench_sum_node || true"

sim_stop: stop

stop:
	@echo "Stopping SAUVC-RPI control-stack nodes..."
	-@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec orca /bin/bash -lc "\
		$(STOP_STACK); \
		source /opt/ros/humble/setup.bash; \
		ros2 daemon stop || true"
	@echo "Stopped."
	@$(MAKE) --no-print-directory stack_status

stack_status:
	@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec -T orca /bin/bash -lc "\
		n=\$$(ps -eo cmd | grep -c '[r]pi_ros2_ws/install'); \
		echo \"control-stack processes running: \$$n\"; \
		if [ \"\$$n\" -gt 0 ]; then ps -eo pid,cmd | grep '[r]pi_ros2_ws/install' | awk '{print \$$1, \$$3}'; fi"

sim_status:
	@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec orca /bin/bash -lc "\
		$(ROS_SETUP) \
		echo '--- Nodes ---'; \
		ros2 node list 2>/dev/null | sort -u | grep -E 'gui_node|supervisor_node|wrench_sum_node|wrench_to_individual|pid_controller|output_sink_force|float32_to_float64|imu_to_orientation|web_video_server' || true; \
		echo ''; \
		echo '--- Key topics ---'; \
		ros2 topic list | grep -E 'wrench_sources/(gui|depth|decision)|wrench_command|thruster_[0-7]/force_N|state/depth_m|targets/depth_m|system_manager/(mode|status)' || true; \
		echo ''; \
		echo '--- Lifecycle ---'; \
		for node in \
			/orca_auv/wrench_sum_node \
			/orca_auv/depth_pid_controller_node \
			/orca_auv/x_coordinate_pid_controller_node \
			/orca_auv/y_coordinate_pid_controller_node \
			/orca_auv/yaw_angle_pid_controller_node; do \
			printf '%s: ' \$$node; \
			timeout 10s ros2 lifecycle get \$$node || true; \
		done"

sim_check:
	@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec orca /bin/bash -lc "\
		$(ROS_SETUP) \
		echo '=== GUI -> wrench_sum ==='; \
		ros2 topic info -v /$(ORCA_NAMESPACE)/control/wrench_sources/gui || true; \
		echo ''; \
		echo '=== Autonomy (decision) -> wrench_sum ==='; \
		ros2 topic info -v /$(ORCA_NAMESPACE)/control/wrench_sources/decision || true; \
		echo ''; \
		echo '=== wrench_sum -> allocator ==='; \
		ros2 topic info -v /$(ORCA_NAMESPACE)/control/wrench_command || true; \
		echo ''; \
		echo '=== allocator -> simulation bridge ==='; \
		ros2 topic info -v /$(ORCA_NAMESPACE)/thrusters/thruster_4/force_N || true"

sim_logs: compose_up
	@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec orca /bin/bash -lc "\
		tail -n 200 /tmp/sauvc_rpi_sim_control.log || true"

sim_gui_detached: compose_up
	@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec -d orca /bin/bash -lc "\
		$(ROS_SETUP) \
		ros2 run gui gui_node \
			--ros-args \
			-r __ns:=/orca_auv \
			-r control/wrench_command:=control/wrench_sources/gui"

sim_wrench_sum_detached: compose_up
	@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec -d orca /bin/bash -lc "\
		$(ROS_SETUP) \
		ros2 launch wrench_sum wrench_sum.launch.py namespace:=orca_auv"
	@sleep 2
	@$(MAKE) sim_activate_wrench_sum

sim_thruster_allocator_detached: compose_up
	@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec -d orca /bin/bash -lc "\
		$(ROS_SETUP) \
		ros2 run thrusters wrench_to_individual_thrusters_output_forces_node \
			--ros-args \
			-r __ns:=/orca_auv"

# --------------------------------------------------------------------
# 重構輔助：參數快照與 bag
# --------------------------------------------------------------------
.PHONY: dump_params snapshot bag_list bag_info

# 把執行中節點的參數全部 dump 回 YAML。
# 池邊調了兩小時的成果不應該靠手抄 —— 調完先跑這個，再把值抄回 config。
dump_params:
	@mkdir -p $(SNAPSHOT_DIR)
	@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec -T orca /bin/bash -lc "\
		$(ROS_SETUP) \
		for n in \$$(ros2 node list 2>/dev/null | grep '^/$(ORCA_NAMESPACE)/' | LC_ALL=C sort -u); do \
			echo \"# ===== \$$n =====\"; \
			timeout 20 ros2 param dump \$$n 2>/dev/null || echo '#   (dump failed)'; \
		done" > $(SNAPSHOT_DIR)/params.yaml
	@echo "Wrote $(SNAPSHOT_DIR)/params.yaml ($$(grep -c '^# =====' $(SNAPSHOT_DIR)/params.yaml) nodes)"

# 節點／topic／service 快照。重構期間用來跟 docs/baseline/ 做差集比對，
# 確認刪掉的剛好就是預期要刪的，沒有非預期缺漏。
snapshot:
	@mkdir -p $(SNAPSHOT_DIR)
	@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec -T orca /bin/bash -lc "\
		$(ROS_SETUP) ros2 node list 2>/dev/null | LC_ALL=C sort -u" > $(SNAPSHOT_DIR)/nodes.txt
	@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec -T orca /bin/bash -lc "\
		$(ROS_SETUP) ros2 topic list 2>/dev/null | LC_ALL=C sort -u" > $(SNAPSHOT_DIR)/topics.txt
	@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec -T orca /bin/bash -lc "\
		$(ROS_SETUP) ros2 service list 2>/dev/null | LC_ALL=C sort -u" > $(SNAPSHOT_DIR)/services.txt
	@echo "nodes=$$(wc -l < $(SNAPSHOT_DIR)/nodes.txt) topics=$$(wc -l < $(SNAPSHOT_DIR)/topics.txt) services=$$(wc -l < $(SNAPSHOT_DIR)/services.txt)"
	@echo "Diff against baseline:  diff ../docs/baseline/nodes.txt $(SNAPSHOT_DIR)/nodes.txt"

bag_list:
	@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec -T orca /bin/bash -lc "\
		du -sh /root/bags/*/ 2>/dev/null | sort -k2 || echo '(no bags yet)'; \
		echo ''; df -h /root/bags | tail -1"

# 用法：make bag_info BAG=orca_20260803_101530（省略則取最新一包）
# AUV 靠 kill switch 斷電時 metadata.yaml 不會寫出，先跑：
#   ros2 bag reindex <bag_dir> -s mcap
bag_info:
	@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec -T orca /bin/bash -lc "\
		$(ROS_SETUP) \
		d=\"\"; \
		if [ -n \"$(BAG)\" ] && [ -d \"/root/bags/$(BAG)\" ]; then d=\"/root/bags/$(BAG)\"; \
		else d=\$$(ls -dt /root/bags/*/ 2>/dev/null | head -1); fi; \
		if [ -z \"\$$d\" ]; then echo '(no bags yet)'; exit 0; fi; \
		echo \"bag: \$$d\"; ros2 bag info \"\$$d\""
