# Developer entry points (bench tools; values are bench values, not platform values).
# Run `make` for the list. Builds use at most 2 parallel jobs.
SHELL := /bin/bash
REPO := $(abspath .)
SIM := $(REPO)/tools/manual_sim
LOG ?= $(shell ls -t $(REPO)/build/manual_sim/log/*/*.ulg 2>/dev/null | head -1)

.PHONY: help px4-sitl px4-v6x ros test sim sim-keys sim-headless sim-stop flight-check

help:
	@echo "make px4-sitl       build the PX4 SITL firmware (firmware/PX4-Autopilot)"
	@echo "make px4-v6x        build the fmu-v6x multicopter firmware"
	@echo "make ros            build the ROS 2 workspace"
	@echo "make test           ROS 2 unit tests of the packages"
	@echo "make sim            manual-flight SITL, stick from the QGroundControl virtual joystick"
	@echo "make sim-keys       manual-flight SITL, stick from the keyboard"
	@echo "make sim-headless   manual-flight SITL without the Gazebo window (QGroundControl stick)"
	@echo "make sim-stop       stop every simulator process"
	@echo "make flight-check   feel metrics of the latest manual flight (LOG=path.ulg for another)"

px4-sitl:
	cd firmware/PX4-Autopilot && MAKEFLAGS=-j2 $(MAKE) px4_sitl_default

px4-v6x:
	cd firmware/PX4-Autopilot && MAKEFLAGS=-j2 $(MAKE) px4_fmu-v6x_multicopter

ros:
	source /opt/ros/jazzy/setup.bash && cd ros2_ws && colcon build --parallel-workers 2

test:
	source /opt/ros/jazzy/setup.bash && cd ros2_ws && \
	  colcon test --parallel-workers 2 --packages-select obstacle_sector_map px4_obstacle_bridge livox_ray_source obstacle_sector_node && \
	  colcon test-result --verbose

sim:
	JOY=qgc bash $(SIM)/fly.sh

sim-keys:
	JOY=keys bash $(SIM)/fly.sh

sim-headless:
	JOY=qgc GUI=0 bash $(SIM)/fly.sh

sim-stop:
	bash $(SIM)/stop.sh

flight-check:
	@if [ -z "$(LOG)" ]; then echo "no manual-flight log found in build/manual_sim/log"; exit 1; fi
	python3 $(SIM)/flight_check.py $(LOG)
