#!/bin/bash

# Experiment 2d: walk with a constant reference velocity (vx, vy, wz) and check
# how well the robot tracks it.

# ------------------------- experiment parameters -------------------------
ALGORITHM="vision"      # wbic | vision | dcm -> config/locomotion_controller.yaml
VEL_CMD_FRAME="global"   # local | global -> config/locomotion_controller.yaml

# Initial state -> config/simulation.yaml
KIN_SCHEME="<<"         # >> | << | >< | <> -> init_motor_angles

# ROS parameters of exp2d (must be float literals, e.g. 1.0, not 1)
BODY_Z=0.2
GAIT_OFFSETS="[0.0, 0.5, 0.5, 0.0]"
T_SW=0.26
T_ST=0.35
STRIDE_HEIGHT=0.06
REF_VELOCITY_X=0.3      # m/s
REF_VELOCITY_Y=0.0      # m/s
REF_VELOCITY_Z=0.5      # rad/s, yaw rate
WALK_TIME=20.0
RAMP_TIME=1.0

STARTUP_DELAY=5         # pause between run.sh start and exp2d start [s]
# -------------------------------------------------------------------------

set -euo pipefail

source "$(dirname -- "$0")/common.sh"

require_float BODY_Z T_SW T_ST STRIDE_HEIGHT REF_VELOCITY_X REF_VELOCITY_Y REF_VELOCITY_Z WALK_TIME RAMP_TIME
require_float_list GAIT_OFFSETS
setup_configs "$ALGORITHM" "$VEL_CMD_FRAME"
setup_kin_scheme "$KIN_SCHEME"

ros_params=(
    body_z:="$BODY_Z"
    gait_offsets:="$GAIT_OFFSETS"
    t_sw:="$T_SW"
    t_st:="$T_ST"
    stride_height:="$STRIDE_HEIGHT"
    ref_velocity_x:="$REF_VELOCITY_X"
    ref_velocity_y:="$REF_VELOCITY_Y"
    ref_velocity_z:="$REF_VELOCITY_Z"
    walk_time:="$WALK_TIME"
    ramp_time:="$RAMP_TIME"
)

# The scenario lasts 12.5 s + walk_time + ramp_time (phases of exp2d_linear.py).
max_duration="$(awk -v w="$WALK_TIME" -v r="$RAMP_TIME" 'BEGIN { print 12.5 + w + r }')"

gait="$(tr -d '[] ' <<< "$GAIT_OFFSETS" | tr ',' '_')"

run_experiment exp2d \
    "${ALGORITHM}_${VEL_CMD_FRAME}_z${BODY_Z}_g${gait}_tsw${T_SW}_tst${T_ST}_h${STRIDE_HEIGHT}_vx${REF_VELOCITY_X}_vy${REF_VELOCITY_Y}_wz${REF_VELOCITY_Z}_walk${WALK_TIME}_ramp${RAMP_TIME}" \
    "$max_duration" "${ros_params[@]}"
