#!/bin/bash

set -euo pipefail
set +m

SCRIPT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"
IMU="${SCRIPT_DIR}/BHI360_IMU/build/bhi360_imu"
CONTACT_SENSOR="${SCRIPT_DIR}/ContactSensor/build/contact_sensor"
STATE_ESTIMATOR="${SCRIPT_DIR}/StateEstimator/build/state_estimator"
LOGGER="${SCRIPT_DIR}/MorsLogger/build/mors_logger"

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

logging_enabled=false
log_dir=""
log_time=120
usage="Usage: $0 [--log] [--log-dir DIR] [--log-time SEC]"

while [ $# -gt 0 ]; do
    case "$1" in
        --log)
            logging_enabled=true
            ;;
        --log-dir)
            if [ $# -lt 2 ] || [ -z "$2" ]; then
                echo "Missing directory for --log-dir" >&2
                echo "$usage" >&2
                exit 1
            fi
            logging_enabled=true
            log_dir="$2"
            shift
            ;;
        --log-time)
            if [ $# -lt 2 ] || ! [[ "$2" =~ ^[0-9]+([.][0-9]+)?$ ]] || [[ "$2" =~ ^0+([.]0+)?$ ]]; then
                echo "--log-time expects a positive number of seconds" >&2
                echo "$usage" >&2
                exit 1
            fi
            logging_enabled=true
            log_time="$2"
            shift
            ;;
        -h|--help)
            echo "$usage"
            exit 0
            ;;
        *)
            echo "Unknown option: $1" >&2
            echo "$usage" >&2
            exit 1
            ;;
    esac
    shift
done

if [ "$logging_enabled" = true ]; then
    check_executable "$LOGGER" "MorsLogger"
fi

process_groups=()
helper_groups=()
LAST_PGID=""

start_component() {
    local name="$1"
    shift

    setsid -- env --default-signal=INT,QUIT,TERM "$@" &
    local pgid=$!

    process_groups+=("$pgid")
    LAST_PGID="$pgid"
    echo "Started ${name}: PGID=${pgid}"
}

cleanup() {
    local status=$?
    local pgid

    trap - EXIT INT TERM

    for pgid in "${helper_groups[@]}"; do
        kill -TERM -- "-${pgid}" 2>/dev/null || true
    done

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
core_process_groups=("${process_groups[@]}")

if [ "$logging_enabled" = true ]; then
    start_component "MorsLogger" "$LOGGER" ${log_dir:+"$log_dir"}
    logger_pgid="$LAST_PGID"

    setsid -- env --default-signal=INT,QUIT,TERM bash -c '
        sleep "$2"
        if kill -INT -- "-$1" 2>/dev/null; then
            echo "[MorsLogger]: stopped after $2 seconds"
        fi
    ' bash "$logger_pgid" "$log_time" &
    helper_groups+=("$!")
fi

wait -n "${core_process_groups[@]}"
