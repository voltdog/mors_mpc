#!/bin/bash

# Experiment 1: straight-line walking along X with the given gait parameters.

# ------------------------- experiment parameters -------------------------
ALGORITHM="vision"      # wbic | vision | dcm -> config/locomotion_controller.yaml
VEL_CMD_FRAME="local"   # local | global -> config/locomotion_controller.yaml

# ROS parameters of exp1 (must be float literals, e.g. 1.0, not 1)
BODY_Z=0.2
T_SW=0.26
T_ST=0.35
STRIDE_HEIGHT=0.06
REF_VELOCITY_X=0.6
WALK_TIME=10.0
RAMP_TIME=2.0

STARTUP_DELAY=5         # pause between run.sh start and exp1 start [s]
# -------------------------------------------------------------------------

set -euo pipefail

source "$(dirname -- "$0")/common.sh"

require_float BODY_Z T_SW T_ST STRIDE_HEIGHT REF_VELOCITY_X WALK_TIME RAMP_TIME
setup_configs "$ALGORITHM" "$VEL_CMD_FRAME"

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

run_experiment exp1 \
    "${ALGORITHM}_${VEL_CMD_FRAME}_z${BODY_Z}_tsw${T_SW}_tst${T_ST}_h${STRIDE_HEIGHT}_vx${REF_VELOCITY_X}_walk${WALK_TIME}_ramp${RAMP_TIME}" \
    "$max_duration" "${ros_params[@]}"
