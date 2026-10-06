#!/bin/bash

# Experiment 5 (down, hardware): straight-line walking along X down a stepped
# pyramid, the robot starts on its top platform (scenario of exp1).

# ------------------------- experiment parameters -------------------------
ALGORITHM="vision"      # wbic | vision | dcm -> config/locomotion_controller.yaml
VEL_CMD_FRAME="local"   # local | global -> config/locomotion_controller.yaml

# Real stairs: description only, goes to params.txt and the log folder name
STAIRS_STEP_HEIGHT=0.05         # m
STAIRS_STEP_LENGTH=0.12          # m, horizontal step inset
STAIRS_STEP_COUNT=6             # integer
STAIRS_TOP_PLATFORM_SIZE=1.0    # m, side of the square top platform
STAIRS_CENTER_POS=0.0            # m, X of the pyramid center

# ROS parameters of exp1 (must be float literals, e.g. 1.0, not 1)
BODY_Z=0.2
T_SW=0.26
T_ST=0.35
STRIDE_HEIGHT=0.06
REF_VELOCITY_X=0.3
WALK_TIME=7.0
RAMP_TIME=1.0

STARTUP_DELAY=5         # pause between run.sh start and exp1 start [s]
# -------------------------------------------------------------------------

set -euo pipefail

source "$(dirname -- "$0")/common.sh"

require_float STAIRS_STEP_HEIGHT STAIRS_STEP_LENGTH STAIRS_TOP_PLATFORM_SIZE STAIRS_CENTER_POS \
    BODY_Z T_SW T_ST STRIDE_HEIGHT REF_VELOCITY_X WALK_TIME RAMP_TIME

if ! [[ "$STAIRS_STEP_COUNT" =~ ^[1-9][0-9]*$ ]]; then
    echo "STAIRS_STEP_COUNT=${STAIRS_STEP_COUNT} must be a positive integer" >&2
    exit 1
fi

setup_configs "$ALGORITHM" "$VEL_CMD_FRAME"

config_params+=(
    stairs_step_height="$STAIRS_STEP_HEIGHT"
    stairs_step_length="$STAIRS_STEP_LENGTH"
    stairs_step_count="$STAIRS_STEP_COUNT"
    stairs_top_platform_size="$STAIRS_TOP_PLATFORM_SIZE"
    stairs_center_pos="$STAIRS_CENTER_POS"
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

SCENARIO=exp1 run_experiment exp5_down \
    "${ALGORITHM}_${VEL_CMD_FRAME}_sh${STAIRS_STEP_HEIGHT}_sl${STAIRS_STEP_LENGTH}_sn${STAIRS_STEP_COUNT}_st${STAIRS_TOP_PLATFORM_SIZE}_sc${STAIRS_CENTER_POS}_z${BODY_Z}_tsw${T_SW}_tst${T_ST}_h${STRIDE_HEIGHT}_vx${REF_VELOCITY_X}_walk${WALK_TIME}_ramp${RAMP_TIME}" \
    "$max_duration" "${ros_params[@]}"
