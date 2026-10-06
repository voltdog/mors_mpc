#!/bin/bash

# Experiment 4 (down, hardware): straight-line walking along X down a trapezoidal
# ramp, the robot starts on its flat top (scenario of exp1).

# ------------------------- experiment parameters -------------------------
ALGORITHM="vision"      # wbic | vision | dcm -> config/locomotion_controller.yaml
VEL_CMD_FRAME="local"   # local | global -> config/locomotion_controller.yaml

# Real ramp: description only, goes to params.txt and the log folder name
RAMP_SLOPE_ANGLE=15.0   # deg
RAMP_HEIGHT=0.4         # m
RAMP_FLAT_LENGTH=2.0    # m, flat top section
RAMP_CENTER_POS=0.0      # m, X of the ramp center

# ROS parameters of exp1 (must be float literals, e.g. 1.0, not 1)
BODY_Z=0.2
T_SW=0.26
T_ST=0.35
STRIDE_HEIGHT=0.06
REF_VELOCITY_X=0.3
WALK_TIME=10.0
RAMP_TIME=1.0           # velocity ramp-up/down [s]

STARTUP_DELAY=5         # pause between run.sh start and exp1 start [s]
# -------------------------------------------------------------------------

set -euo pipefail

source "$(dirname -- "$0")/common.sh"

require_float RAMP_SLOPE_ANGLE RAMP_HEIGHT RAMP_FLAT_LENGTH RAMP_CENTER_POS \
    BODY_Z T_SW T_ST STRIDE_HEIGHT REF_VELOCITY_X WALK_TIME RAMP_TIME

setup_configs "$ALGORITHM" "$VEL_CMD_FRAME"

config_params+=(
    ramp_slope_angle="$RAMP_SLOPE_ANGLE"
    ramp_height="$RAMP_HEIGHT"
    ramp_flat_length="$RAMP_FLAT_LENGTH"
    ramp_center_pos="$RAMP_CENTER_POS"
)

ros_params=(
    body_z:="$BODY_Z"
    t_sw:="$T_SW"
    t_st:="$T_ST"
    stride_height:="$STRIDE_HEIGHT"
    ref_velocity_x:="$REF_VELOCITY_X"
    walk_time:="$WALK_TIME"
    ramp_time:="$RAMP_TIME"
)

# The scenario lasts 12.5 s + walk_time + ramp_time (phases of exp1_max_t_sw.py).
max_duration="$(awk -v w="$WALK_TIME" -v r="$RAMP_TIME" 'BEGIN { print 12.5 + w + r }')"

SCENARIO=exp1 run_experiment exp4_down \
    "${ALGORITHM}_${VEL_CMD_FRAME}_ra${RAMP_SLOPE_ANGLE}_rh${RAMP_HEIGHT}_rl${RAMP_FLAT_LENGTH}_rc${RAMP_CENTER_POS}_z${BODY_Z}_tsw${T_SW}_tst${T_ST}_h${STRIDE_HEIGHT}_vx${REF_VELOCITY_X}_walk${WALK_TIME}_ramp${RAMP_TIME}" \
    "$max_duration" "${ros_params[@]}"
