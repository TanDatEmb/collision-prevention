# Developer entry points (bench tools; values are bench values, not platform values).
# Run `make` for the list. Builds use at most 2 parallel jobs.
SHELL := /bin/bash
REPO := $(abspath .)
SIM := $(REPO)/tools/manual_sim
LOG ?= $(shell ls -t $(REPO)/build/manual_sim/log/*/*.ulg 2>/dev/null | head -1)
LIVOX_IFACE ?= enp52s0
LIVOX_HOST_IP ?= 192.168.1.5
LIVOX_SENSOR_IP ?= 192.168.1.100
LIVOX_CONFIG ?= $(REPO)/config/livox/mid360s_config.json
LIVOX_DURATION_S ?= 10
LIVOX_PRIMARY_SN ?=
LIVOX_ALLOW_UNSYNC ?= 0

.PHONY: help px4-sitl px4-v6x ros test dashboard sim sim-dashboard sim-keys sim-headless sim-stop flight-check livox-up livox-check livox-down

help:
	@echo "make px4-sitl       build the PX4 SITL firmware (firmware/PX4-Autopilot)"
	@echo "make px4-v6x        build the fmu-v6x multicopter firmware"
	@echo "make ros            build the ROS 2 workspace"
	@echo "make test           ROS 2 unit tests of the packages"
	@echo "make sim            manual-flight SITL, stick from the QGroundControl virtual joystick"
	@echo "make sim-dashboard  manual-flight SITL with the live CP Monitor at http://127.0.0.1:8765"
	@echo "make sim-keys       manual-flight SITL, stick from the keyboard"
	@echo "make sim-headless   manual-flight SITL without the Gazebo window (QGroundControl stick)"
	@echo "make sim-stop       stop every simulator process"
	@echo "make dashboard      serve the CP Monitor and wait for a SITL connection"
	@echo "make flight-check   feel metrics of the latest manual flight (LOG=path.ulg for another)"
	@echo "make livox-up       assign the Livox Ethernet IP and start the software PTP master"
	@echo "make livox-check    run the project-linked SDK2 live probe (LIVOX_DURATION_S=10)"
	@echo "make livox-down     stop only the PTP master started by livox-up"

px4-sitl:
	cd firmware/PX4-Autopilot && MAKEFLAGS=-j2 $(MAKE) px4_sitl_default

px4-v6x:
	cd firmware/PX4-Autopilot && MAKEFLAGS=-j2 $(MAKE) px4_fmu-v6x_multicopter

ros:
	source /opt/ros/jazzy/setup.bash && cd ros2_ws && colcon build --parallel-workers 2

test:
	source /opt/ros/jazzy/setup.bash && cd ros2_ws && \
	  colcon test --parallel-workers 2 --packages-select collision_prevention_msgs obstacle_sector_map px4_obstacle_bridge livox_ray_source obstacle_sector_node && \
	  colcon test-result --verbose

dashboard:
	source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash && \
	  python3 tools/dashboard/bridge.py --open

sim:
	JOY=qgc bash $(SIM)/fly.sh

sim-dashboard:
	JOY=qgc DASHBOARD=1 DASHBOARD_OPEN=1 bash $(SIM)/fly.sh

sim-keys:
	JOY=keys bash $(SIM)/fly.sh

sim-headless:
	JOY=qgc GUI=0 bash $(SIM)/fly.sh

sim-stop:
	bash $(SIM)/stop.sh

flight-check:
	@if [ -z "$(LOG)" ]; then echo "no manual-flight log found in build/manual_sim/log"; exit 1; fi
	python3 $(SIM)/flight_check.py $(LOG)

livox-up:
	LIVOX_IFACE="$(LIVOX_IFACE)" LIVOX_HOST_IP="$(LIVOX_HOST_IP)" \
	  bash tools/livox/livox_up.sh

livox-check:
	LIVOX_IFACE="$(LIVOX_IFACE)" LIVOX_HOST_IP="$(LIVOX_HOST_IP)" \
	  LIVOX_SENSOR_IP="$(LIVOX_SENSOR_IP)" LIVOX_CONFIG="$(LIVOX_CONFIG)" \
	  LIVOX_DURATION_S="$(LIVOX_DURATION_S)" LIVOX_PRIMARY_SN="$(LIVOX_PRIMARY_SN)" \
	  LIVOX_ALLOW_UNSYNC="$(LIVOX_ALLOW_UNSYNC)" bash tools/livox/livox_check.sh

livox-down:
	LIVOX_RUNTIME_DIR="$(REPO)/build/livox" bash tools/livox/livox_down.sh
