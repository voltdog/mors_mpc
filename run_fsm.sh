#!/bin/bash

set -e
set +m

cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
export LCM_LOCOMOTION_URL='udpm://239.255.76.67:17673?ttl=0'
export LCM_SERVO_URL='udpm://239.255.76.68:17674?ttl=0'
export CONFIGPATH="$PWD/config"

process_groups=()

start_component() {
    setsid -- env --default-signal=INT,QUIT,TERM "$@" &
    process_groups+=("$!")
}

signal_groups() {
    local signal="$1" pgid target
    for pgid in "${process_groups[@]}"; do
        target="-$pgid"
        if [[ $signal == INT && $pgid == "$ros_launch_pid" ]]; then
            # ROS launch forwards SIGINT to its child.
            target="$pgid"
        fi
        kill -s "$signal" -- "$target" 2>/dev/null || true
    done
}

groups_are_running() {
    local pgid
    for pgid in "${process_groups[@]}"; do
        if kill -0 -- "-$pgid" 2>/dev/null; then
            return 0
        fi
    done
    return 1
}

cleanup() {
    trap '' INT TERM
    signal_groups INT
    if groups_are_running; then
        sleep 2
        signal_groups TERM
    fi
    if groups_are_running; then
        sleep 1
        signal_groups KILL
    fi
    wait || true
}

trap 'exit 130' INT
trap 'exit 143' TERM
trap cleanup EXIT

start_component .mpc_venv/bin/python Simulator/mors_simulator.py
start_component ./FSMLocomotion/build/fsm_locomotion_node
start_component ros2 launch ros2lcm_bridge bridge.launch.py "lcm_url:=$LCM_LOCOMOTION_URL"
ros_launch_pid=$!

wait -n
