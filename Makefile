# Define variables
IMAGE_NAME := orca-auv-rpi-ros2-image
CONTAINER_NAME := orca-auv-rpi-ros2-container
WORKSPACE := rpi_ros2_ws
IMAGE_OWNER_NAME := dianyueguo
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
ROS_NET_ENV := ROS_DOMAIN_ID=$(ROS_DOMAIN_ID) ROS_LOCALHOST_ONLY=$(ROS_LOCALHOST_ONLY) RMW_IMPLEMENTATION=$(RMW_IMPLEMENTATION)

.PHONY: all debug compose_up compose_down compose_build compose_shell init launch launch_debug launch_detached compose_init compose_launch compose_launch_detached compose_clean clean update_image

all: init launch

debug: init
	@echo "Starting tile-line debug viewer..."
	@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec -d orca /bin/bash -lc "\
		$(ROS_SETUP) \
		sleep 5; \
		export XDG_RUNTIME_DIR=/tmp/runtime-root; \
		mkdir -p \$$XDG_RUNTIME_DIR; \
		chmod 700 \$$XDG_RUNTIME_DIR; \
		export LIBGL_ALWAYS_SOFTWARE=1; \
		export QT_X11_NO_MITSHM=1; \
		exec rqt_image_view /orca_auv/camera/bottom/debug/tile_lines"
	@echo "Starting hardware launch with tile-line debug image publishing enabled..."
	HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec orca /bin/bash -lc "\
		$(ROS_SETUP) \
		ros2 launch src/launch/orca_bringup.launch.py publish_lk_debug_image:=true"

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
		ros2 launch src/launch/orca_bringup.launch.py"

launch_debug: compose_up
	HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec orca /bin/bash -lc "\
		$(ROS_SETUP) \
		ros2 launch src/launch/orca_bringup.launch.py publish_lk_debug_image:=true"

launch_detached: compose_up
	@echo "Launching ROS stack in detached mode"
	HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec -d orca /bin/bash -lc "\
		cd $(WORKSPACE) && \
		source /opt/ros/humble/setup.bash && \
		source /root/uros_ws/install/local_setup.bash && \
		source install/setup.bash && \
		ros2 launch src/launch/orca_bringup.launch.py"

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
# - bottom_camera_node
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
	sim_set_manual sim_thruster_allocator_detached sim_lk_detached \
	sim_rqt_lk sim_rqt_tile_lines

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
	@echo ""
	@echo "To view tile-line tracking:"
	@echo "  make sim_rqt_tile_lines"

sim_launch_detached: compose_up
	@echo "Stopping old SAUVC-RPI simulation-control nodes..."
	-@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec orca /bin/bash -lc "\
		pkill -f '[s]imulation_control.launch.py' || true; \
		pkill -f '[b]ottom_camera_pid_fbc_launch.py' || true; \
		pkill -f '[d]epth_control_launch.py' || true; \
		pkill -f '[w]rench_sum.launch.py' || true; \
		pkill -f '[g]ui_node' || true; \
		pkill -f '[s]upervisor_node' || true; \
		pkill -f '[w]rench_sum_node' || true; \
		pkill -f '[w]rench_to_individual_thrusters_output_forces_node' || true; \
		pkill -f '[l]k_total_transform_node' || true; \
		pkill -f '[g]eneric_pid_controller_node' || true; \
		pkill -f '[b]ottom_camera_pid_bridge_node' || true; \
		pkill -f '[w]aypoint_target_publisher' || true; \
		pkill -f '[y]aw_reference_unwrapper_node' || true; \
		pkill -f '[o]utput_sink_force_to_output_wrench_node' || true; \
		pkill -f '[f]loat32_to_float64_converter_node' || true; \
		pkill -f '[i]mu_to_orientation_node' || true; \
		pkill -f '[b]ottom_camera_node' || true; \
		pkill -f '[o]rca_bringup.launch.py' || true; \
		pkill -f '[w]eb_video_server' || true; \
		source /opt/ros/humble/setup.bash; \
		ros2 daemon stop || true; \
		sleep 1"
	@echo "Starting simulation launch: GUI, tile-line tracking, supervisor, controllers, wrench sum, thruster force allocator..."
	@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec -d orca /bin/bash -lc "\
		$(ROS_SETUP) \
		exec ros2 launch src/launch/simulation_control.launch.py \
			namespace:=orca_auv \
			publish_lk_debug_image:=true \
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

sim_stop:
	@echo "Stopping SAUVC-RPI simulation-control nodes..."
	-@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec orca /bin/bash -lc "\
		pkill -f '[s]imulation_control.launch.py' || true; \
		pkill -f '[b]ottom_camera_pid_fbc_launch.py' || true; \
		pkill -f '[d]epth_control_launch.py' || true; \
		pkill -f '[w]rench_sum.launch.py' || true; \
		pkill -f '[g]ui_node' || true; \
		pkill -f '[s]upervisor_node' || true; \
		pkill -f '[w]rench_sum_node' || true; \
		pkill -f '[w]rench_to_individual_thrusters_output_forces_node' || true; \
		pkill -f '[l]k_total_transform_node' || true; \
		pkill -f '[g]eneric_pid_controller_node' || true; \
		pkill -f '[b]ottom_camera_pid_bridge_node' || true; \
		pkill -f '[w]aypoint_target_publisher' || true; \
		pkill -f '[y]aw_reference_unwrapper_node' || true; \
		pkill -f '[o]utput_sink_force_to_output_wrench_node' || true; \
		pkill -f '[f]loat32_to_float64_converter_node' || true; \
		pkill -f '[i]mu_to_orientation_node' || true; \
		pkill -f '[b]ottom_camera_node' || true; \
		pkill -f '[o]rca_bringup.launch.py' || true; \
		pkill -f '[w]eb_video_server' || true; \
		source /opt/ros/humble/setup.bash; \
		ros2 daemon stop || true; \
		sleep 1"
	@echo "Stopped."

sim_status:
	@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec orca /bin/bash -lc "\
		$(ROS_SETUP) \
		echo '--- Nodes ---'; \
		ros2 node list 2>/dev/null | sort -u | grep -E 'gui_node|supervisor_node|wrench_sum_node|wrench_to_individual|lk_total_transform|pid_controller|bottom_camera_pid_bridge|waypoint_target|yaw_reference|output_sink_force|float32_to_float64|imu_to_orientation|web_video_server' || true; \
		echo ''; \
		echo '--- Key topics ---'; \
		ros2 topic list | grep -E 'wrench_sources/(gui|bottom_camera|depth)|wrench_command|thruster_[0-7]/force_N|debug/tile_lines|camera/bottom/(image_raw|pose_px)|state/depth_m|targets/depth_m|system_manager/(mode|status)' || true; \
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
		ros2 topic info -v /orca_auv/control/wrench_sources/gui || true; \
		echo ''; \
		echo '=== wrench_sum -> allocator ==='; \
		ros2 topic info -v /orca_auv/control/wrench_command || true; \
		echo ''; \
		echo '=== allocator -> simulation bridge ==='; \
		ros2 topic info -v /orca_auv/thrusters/thruster_4/force_N || true; \
		echo ''; \
		echo '=== Tile-line debug ==='; \
		timeout 5s ros2 topic hz /orca_auv/camera/bottom/debug/tile_lines --window 10 || true"

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

sim_lk_detached: compose_up
	@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec -d orca /bin/bash -lc "\
		$(ROS_SETUP) \
		ros2 run xy_control lk_total_transform_node \
			--ros-args \
			-r __ns:=/orca_auv \
			-p publish_debug_image:=true \
			-p image_topic:=camera/bottom/image_raw"

sim_rqt_lk: sim_rqt_tile_lines

sim_rqt_tile_lines: compose_up
	@HOST_DISPLAY=$(HOST_DISPLAY) XAUTH_FILE=$(XAUTH_FILE) XAUTHORITY=$(XAUTHORITY) $(ROS_NET_ENV) $(COMPOSE) exec orca /bin/bash -lc "\
		$(ROS_SETUP) \
		export XDG_RUNTIME_DIR=/tmp/runtime-root; \
		mkdir -p \$$XDG_RUNTIME_DIR; \
		chmod 700 \$$XDG_RUNTIME_DIR; \
		export LIBGL_ALWAYS_SOFTWARE=1; \
		export QT_X11_NO_MITSHM=1; \
		rqt_image_view /orca_auv/camera/bottom/debug/tile_lines"
