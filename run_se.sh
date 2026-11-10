#!/bin/bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
IMU="${SCRIPT_DIR}/BHI360_IMU/build/bhi360_imu"
CONTACT_SENSOR="${SCRIPT_DIR}/ContactSensor/build/contact_sensor"
STATE_ESTIMATOR="${SCRIPT_DIR}/StateEstimator/build/state_estimator"

check_executable() {
    local executable="$1"
    local project="$2"

    if [ ! -x "$executable" ]; then
        echo "Missing executable: ${executable}" >&2
        echo "Build it with: cmake -S ${project} -B ${project}/build && cmake --build ${project}/build -j" >&2
        exit 1
    fi
}

check_executable "$IMU" "BHI360_IMU"
check_executable "$CONTACT_SENSOR" "ContactSensor"
check_executable "$STATE_ESTIMATOR" "StateEstimator"

export CONFIGPATH="${CONFIGPATH:-${SCRIPT_DIR}/config}"

lcm_fallback_url="${LCM_DEFAULT_URL:-${LCM_LOCOMOTION_URL:-}}"
if [ -z "$lcm_fallback_url" ]; then
    echo "LCM_DEFAULT_URL or LCM_LOCOMOTION_URL must be set" >&2
    exit 1
fi

export LCM_LOCOMOTION_URL="${LCM_LOCOMOTION_URL:-$lcm_fallback_url}"
export LCM_SERVO_URL="${LCM_SERVO_URL:-$lcm_fallback_url}"

process_groups=()

start_component() {
    local name="$1"
    local executable="$2"

    setsid -- env --default-signal=INT,QUIT,TERM "$executable" &
    process_groups+=("$!")
    echo "Started ${name}: PGID=$!"
}

cleanup() {
    local status=$?
    local pgid

    trap - EXIT INT TERM

    for pgid in "${process_groups[@]}"; do
        kill -INT -- "-${pgid}" 2>/dev/null || true
    done

    wait 2>/dev/null || true
    exit "$status"
}

trap 'exit 130' INT
trap 'exit 143' TERM
trap cleanup EXIT

start_component "BHI360 IMU" "$IMU"
start_component "contact sensor" "$CONTACT_SENSOR"
start_component "state estimator" "$STATE_ESTIMATOR"

wait -n "${process_groups[@]}"
