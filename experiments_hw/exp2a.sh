#!/bin/bash

# Experiment 2a (hardware): open-loop curvilinear walk on a timer (odom is not
# published while the robot lies) - WALK_TIME along the local X axis, then turns
# in place by TURN_ANGLES with a WALK_TIME walk after each
# (TURN_ANGLES and RAMP_TIME are constants of exp2a_curvilinear_timed.py).

# ------------------------- experiment parameters -------------------------
ALGORITHM="dcm"      # wbic | vision | dcm -> config/locomotion_controller.yaml
VEL_CMD_FRAME="local"   # local | global -> config/locomotion_controller.yaml

# ROS parameters of exp2a (must be float literals, e.g. 1.0, not 1)
BODY_Z=0.2
GAIT_OFFSETS="[0.0, 0.5, 0.5, 0.0]"
T_SW=0.26 # max (sim): vision 0.23 | dcm 0.26
T_ST=0.4 # max (sim): vision 0.35 | dcm 0.4
STRIDE_HEIGHT=0.04
REF_VELOCITY_X=0.27      # m/s
REF_VELOCITY_Z=0.5      # rad/s, yaw rate magnitude
WALK_TIME=6.0           # s, every walk segment, ramps included: length = vx * (WALK_TIME - 1.0)

STARTUP_DELAY=5         # pause between run.sh start and exp2a start [s]
# -------------------------------------------------------------------------

set -euo pipefail

source "$(dirname -- "$0")/common.sh"

require_float BODY_Z T_SW T_ST STRIDE_HEIGHT REF_VELOCITY_X REF_VELOCITY_Z WALK_TIME
require_float_list GAIT_OFFSETS
setup_configs "$ALGORITHM" "$VEL_CMD_FRAME"

ros_params=(
    body_z:="$BODY_Z"
    gait_offsets:="$GAIT_OFFSETS"
    t_sw:="$T_SW"
    t_st:="$T_ST"
    stride_height:="$STRIDE_HEIGHT"
    ref_velocity_x:="$REF_VELOCITY_X"
    ref_velocity_z:="$REF_VELOCITY_Z"
    walk_time:="$WALK_TIME"
)

# 12.5 s of fixed phases + 3 walks x WALK_TIME + 2 turns x (pi/2 / |wz| + RAMP_TIME (1 s)).
if ! max_duration="$(awk -v w="$WALK_TIME" -v wz="$REF_VELOCITY_Z" \
        'BEGIN { if (w < 2.0 || wz == 0) exit 1; if (wz < 0) wz = -wz; print 12.5 + 3 * w + 2 * (atan2(1, 0) / wz + 1.0) }')"; then
    echo "WALK_TIME=${WALK_TIME} must be >= 2.0 (2 x RAMP_TIME), REF_VELOCITY_Z must be non-zero" >&2
    exit 1
fi

gait="$(tr -d '[] ' <<< "$GAIT_OFFSETS" | tr ',' '_')"

SCENARIO=exp2a_timed run_experiment exp2a \
    "${ALGORITHM}_${VEL_CMD_FRAME}_z${BODY_Z}_g${gait}_tsw${T_SW}_tst${T_ST}_h${STRIDE_HEIGHT}_vx${REF_VELOCITY_X}_wz${REF_VELOCITY_Z}_walk${WALK_TIME}" \
    "$max_duration" "${ros_params[@]}"
