#!/usr/bin/env bash
# Run (or attach to) the dddmr:jazzy container for the dddmr navigation stack.
# Usage: scripts/run_dddmr_container.sh [command ...]   (default: interactive bash)
# Example: scripts/run_dddmr_container.sh bash -ic ws_build
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

IMAGE="${DDDMR_IMAGE:-dddmr:jazzy}"
CONTAINER="${DDDMR_CONTAINER:-dddmr_jazzy}"
DDDMR_WS="${DDDMR_WS:-$HOME/dddmr_navigation}"
DDDMR_BAGS="${DDDMR_BAGS:-$HOME/dddmr_bags}"
MORS_DDDMR_PKG="$REPO_DIR/ros_ws/src/mors_dddmr"

# DDS settings must match the host side (see run.sh).
env_args=(
    --env "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
    --env "FASTDDS_BUILTIN_TRANSPORTS=${FASTDDS_BUILTIN_TRANSPORTS:-UDPv4}"
)
for var in RMW_IMPLEMENTATION ROS_AUTOMATIC_DISCOVERY_RANGE ROS_STATIC_PEERS; do
    if [ -n "${!var:-}" ]; then
        env_args+=(--env "$var=${!var}")
    fi
done

tty_args=(-i)
if [ -t 0 ] && [ -t 1 ]; then
    tty_args=(-it)
fi

if [ $# -eq 0 ]; then
    set -- bash
fi

# Source ROS and the dddmr workspace, then run the requested command.
entry='source /opt/ros/jazzy/setup.bash
if [ -f /root/dddmr_navigation/install/setup.bash ]; then
    source /root/dddmr_navigation/install/setup.bash
fi
cd /root/dddmr_navigation
exec "$@"'

if [ -n "$(docker ps -q --filter "name=^${CONTAINER}$")" ]; then
    exec docker exec "${tty_args[@]}" "${env_args[@]}" "$CONTAINER" bash -c "$entry" bash "$@"
fi

volume_args=(
    --volume /tmp:/tmp
    --volume /dev:/dev
    --volume "$DDDMR_WS:/root/dddmr_navigation"
    --volume "$DDDMR_BAGS:/root/dddmr_bags"
)
# Mount the MORS integration package only if it exists, otherwise docker
# would create an empty root-owned directory on the host.
if [ -d "$MORS_DDDMR_PKG" ]; then
    volume_args+=(--volume "$MORS_DDDMR_PKG:/root/dddmr_navigation/src/mors_dddmr")
fi

mkdir -p "$DDDMR_BAGS"
xhost +local:docker > /dev/null 2>&1 || true

exec docker run --rm "${tty_args[@]}" \
    --privileged \
    --network=host \
    --env DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    "${env_args[@]}" \
    "${volume_args[@]}" \
    --name "$CONTAINER" \
    "$IMAGE" \
    bash -c "$entry" bash "$@"
