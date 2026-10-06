#!/bin/bash

# Experiment 2c (hardware): open-loop walk along a circle of radius vx/wz for
# N_LAPS laps (N_LAPS and RAMP_TIME are constants of exp2c_curvilinear.py).

# ------------------------- experiment parameters -------------------------
ALGORITHM="vision"      # wbic | vision | dcm -> config/locomotion_controller.yaml
VEL_CMD_FRAME="local"   # local | global -> config/locomotion_controller.yaml

# ROS parameters of exp2c (must be float literals, e.g. 1.0, not 1)
BODY_Z=0.2
GAIT_OFFSETS="[0.0, 0.5, 0.5, 0.0]"
T_SW=0.18
T_ST=0.35
STRIDE_HEIGHT=0.06
REF_VELOCITY_X=0.0      # m/s
REF_VELOCITY_Z=0.7      # rad/s, > 0 turns left (CCW), must be non-zero

STARTUP_DELAY=5         # pause between run.sh start and exp2c start [s]
# -------------------------------------------------------------------------

set -euo pipefail

source "$(dirname -- "$0")/common.sh"

require_float BODY_Z T_SW T_ST STRIDE_HEIGHT REF_VELOCITY_X REF_VELOCITY_Z
require_float_list GAIT_OFFSETS

# 12.5 s of fixed phases + walk_time (N_LAPS * 2 pi / |wz|, N_LAPS = 1) + RAMP_TIME (1 s) on stop.
if ! max_duration="$(awk -v w="$REF_VELOCITY_Z" \
    'BEGIN { if (w == 0) exit 1; print 13.5 + 2 * atan2(0, -1) / (w < 0 ? -w : w) }')"; then
    echo "REF_VELOCITY_Z must be non-zero" >&2
    exit 1
fi

setup_configs "$ALGORITHM" "$VEL_CMD_FRAME"

ros_params=(
    body_z:="$BODY_Z"
    gait_offsets:="$GAIT_OFFSETS"
    t_sw:="$T_SW"
    t_st:="$T_ST"
    stride_height:="$STRIDE_HEIGHT"
    ref_velocity_x:="$REF_VELOCITY_X"
    ref_velocity_z:="$REF_VELOCITY_Z"
)

gait="$(tr -d '[] ' <<< "$GAIT_OFFSETS" | tr ',' '_')"

run_experiment exp2c \
    "${ALGORITHM}_${VEL_CMD_FRAME}_z${BODY_Z}_g${gait}_tsw${T_SW}_tst${T_ST}_h${STRIDE_HEIGHT}_vx${REF_VELOCITY_X}_wz${REF_VELOCITY_Z}" \
    "$max_duration" "${ros_params[@]}"
