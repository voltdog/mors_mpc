#!/bin/bash

# Experiment 2a (hardware): curvilinear walk closed-loop on odom - SIDE_LENGTH
# along the local X axis, then turns in place by TURN_ANGLES with a SIDE_LENGTH
# walk after each (SIDE_LENGTH and TURN_ANGLES are constants of exp2a_curvilinear.py).

# ------------------------- experiment parameters -------------------------
ALGORITHM="vision"      # wbic | vision | dcm -> config/locomotion_controller.yaml
VEL_CMD_FRAME="local"   # local | global -> config/locomotion_controller.yaml

# ROS parameters of exp2a (must be float literals, e.g. 1.0, not 1)
BODY_Z=0.2
GAIT_OFFSETS="[0.0, 0.5, 0.5, 0.0]"
T_SW=0.26
T_ST=0.35
STRIDE_HEIGHT=0.06
REF_VELOCITY_X=0.3      # m/s
REF_VELOCITY_Z=0.5      # rad/s, yaw rate magnitude

STARTUP_DELAY=5         # pause between run.sh start and exp2a start [s]
# -------------------------------------------------------------------------

set -euo pipefail

source "$(dirname -- "$0")/common.sh"

require_float BODY_Z T_SW T_ST STRIDE_HEIGHT REF_VELOCITY_X REF_VELOCITY_Z
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
)

# WALK and TURN end on the odom target, so the duration is not known in advance.
# Worst case: 12.5 s of fixed phases + 5 motions x MOTION_TIMEOUT (30 s).
max_duration=162.5

gait="$(tr -d '[] ' <<< "$GAIT_OFFSETS" | tr ',' '_')"

run_experiment exp2a \
    "${ALGORITHM}_${VEL_CMD_FRAME}_z${BODY_Z}_g${gait}_tsw${T_SW}_tst${T_ST}_h${STRIDE_HEIGHT}_vx${REF_VELOCITY_X}_wz${REF_VELOCITY_Z}" \
    "$max_duration" "${ros_params[@]}"
