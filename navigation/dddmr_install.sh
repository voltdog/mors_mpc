#!/usr/bin/env bash
# Install the dddmr navigation stack for MORS:
#   - upstream dddmr_navigation in ~/dddmr_navigation, pinned to DDDMR_COMMIT, plus navigation/patches/*.patch;
#   - docker image dddmr:jazzy and the dddmr workspace built in it (with ros_ws/src/mors_dddmr);
#   - dddmr_sys_core interfaces built on the host in ~/dddmr_host_ws (for navigation/tools/nav_goals.py).
# Usage: navigation/dddmr_install.sh [--rebuild-image] [--clean]
#   --rebuild-image  rebuild the docker image even if it exists
#   --clean          remove build/install/log of both workspaces before building
# Safe to rerun: applied patches are skipped, a patch added to navigation/patches is applied.
set -euo pipefail

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
PATCH_DIR="$REPO_ROOT/navigation/patches"
UBUNTU_CODENAME="noble"

DDDMR_URL="https://github.com/dfl-rlab/dddmr_navigation.git"
DDDMR_COMMIT="5027ce6ed4d7b2abac4d9d824cae3e6319da18a2"
DOCKER_DIR="dddmr_docker/docker_file_2404"

# Shared with scripts/run_dddmr_container.sh.
export DDDMR_WS="${DDDMR_WS:-$HOME/dddmr_navigation}"
export DDDMR_BAGS="${DDDMR_BAGS:-$HOME/dddmr_bags}"
export DDDMR_IMAGE="${DDDMR_IMAGE:-dddmr:jazzy}"
export DDDMR_CONTAINER="${DDDMR_CONTAINER:-dddmr_jazzy}"
DDDMR_HOST_WS="${DDDMR_HOST_WS:-$HOME/dddmr_host_ws}"

REBUILD_IMAGE=0
CLEAN=0

log() {
  printf '[dddmr_install] %s\n' "$*"
}

die() {
  printf '[dddmr_install][error] %s\n' "$*" >&2
  exit 1
}

parse_args() {
  while (( $# )); do
    case "$1" in
      --rebuild-image) REBUILD_IMAGE=1 ;;
      --clean) CLEAN=1 ;;
      -h|--help) sed -n '2,9p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
      *) die "Unknown argument: $1" ;;
    esac
    shift
  done
}

check_system() {
  (( EUID != 0 )) || die "Run as a regular user, sudo is requested when needed."
  # shellcheck disable=SC1091
  source /etc/os-release
  [[ "${ID:-}" == "ubuntu" && "${VERSION_ID:-}" == 24.* ]] || die "Ubuntu 24.x is required (found: ${PRETTY_NAME:-unknown})"
  [[ -f /opt/ros/jazzy/setup.bash ]] || die "ROS2 Jazzy was not found in /opt/ros/jazzy. Install it first."
  command -v sudo >/dev/null 2>&1 || die "Required command is missing: sudo"
  sudo -v
}

install_apt_packages() {
  log "Installing apt dependencies..."
  sudo apt-get update
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    git \
    curl \
    ca-certificates \
    x11-xserver-utils \
    python3-colcon-common-extensions
}

install_docker() {
  if command -v docker >/dev/null 2>&1; then
    log "Docker already installed: $(docker --version)"
  else
    log "Installing Docker Engine..."
    sudo install -m 0755 -d /etc/apt/keyrings
    sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
    sudo chmod a+r /etc/apt/keyrings/docker.asc
    printf 'deb [arch=%s signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu %s stable\n' \
      "$(dpkg --print-architecture)" "$UBUNTU_CODENAME" | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null
    sudo apt-get update
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
      docker-ce docker-ce-cli containerd.io docker-buildx-plugin
  fi
  sudo systemctl enable --now docker

  if ! id -nG "$USER" | tr ' ' '\n' | grep -qx docker; then
    log "Adding $USER to the docker group (re-login to use docker without sudo)"
    sudo usermod -aG docker "$USER"
  fi
}

# Until re-login after usermod the shell has no docker group: run the command through sg.
with_docker() {
  if docker info >/dev/null 2>&1; then
    "$@"
  else
    sg docker -c "$(printf '%q ' "$@")"
  fi
}

fetch_dddmr() {
  if [[ ! -e "$DDDMR_WS" ]]; then
    log "Cloning dddmr_navigation into $DDDMR_WS"
    git clone "$DDDMR_URL" "$DDDMR_WS"
  fi
  [[ -d "$DDDMR_WS/.git" ]] || die "Path exists but is not a git repo: $DDDMR_WS"

  local head
  head="$(git -C "$DDDMR_WS" rev-parse HEAD)"
  [[ "$head" != "$DDDMR_COMMIT" ]] || return 0

  if ! tree_matches; then
    git -C "$DDDMR_WS" status --short >&2
    die "$DDDMR_WS is at ${head:0:7}, expected ${DDDMR_COMMIT:0:7}, and has local changes. Revert them and rerun."
  fi
  git -C "$DDDMR_WS" cat-file -e "${DDDMR_COMMIT}^{commit}" 2>/dev/null || git -C "$DDDMR_WS" fetch origin
  log "Checking out dddmr_navigation ${DDDMR_COMMIT:0:7}"
  git -C "$DDDMR_WS" -c advice.detachedHead=false checkout "$DDDMR_COMMIT"
}

# True if the working tree is exactly HEAD with the given patches applied (tracked and untracked files).
tree_matches() {
  local tmp rc=0 p
  tmp="$(mktemp -d)"
  export GIT_INDEX_FILE="$tmp/index"
  git -C "$DDDMR_WS" read-tree HEAD
  for p in "$@"; do
    git -C "$DDDMR_WS" apply --cached "$p" 2>/dev/null || { rc=1; break; }
  done
  if (( rc == 0 )); then
    git -C "$DDDMR_WS" update-index -q --refresh
    git -C "$DDDMR_WS" diff --quiet || rc=1
    [[ -z "$(git -C "$DDDMR_WS" ls-files --others --exclude-standard)" ]] || rc=1
  fi
  unset GIT_INDEX_FILE
  rm -rf "$tmp"
  return "$rc"
}

# Patches are applied in name order. A tree with the first k of them applied (k = 0 on a fresh clone)
# gets the rest; any other local change stops the script.
apply_patches() {
  local patches=() k p
  shopt -s nullglob
  patches=("$PATCH_DIR"/*.patch)
  shopt -u nullglob

  for (( k = ${#patches[@]}; k >= 0; k-- )); do
    tree_matches "${patches[@]:0:k}" || continue
    (( k < ${#patches[@]} )) || log "All patches are already applied"
    for p in "${patches[@]:k}"; do
      log "Applying $(basename "$p")"
      git -C "$DDDMR_WS" apply "$p"
    done
    return 0
  done

  git -C "$DDDMR_WS" status --short >&2
  die "$DDDMR_WS has changes that are not in $PATCH_DIR. Save them as a patch or revert them, then rerun."
}

clean_workspaces() {
  log "Removing build artifacts of $DDDMR_WS and $DDDMR_HOST_WS"
  # The container builds as root.
  sudo rm -rf "$DDDMR_WS"/{build,install,log} "$DDDMR_HOST_WS"/{build,install,log}
}

build_image() {
  if (( ! REBUILD_IMAGE )) && with_docker docker image inspect "$DDDMR_IMAGE" >/dev/null 2>&1; then
    log "Docker image $DDDMR_IMAGE already exists (--rebuild-image to rebuild)"
    return 0
  fi
  log "Building docker image $DDDMR_IMAGE, takes 30-60 min..."
  with_docker docker build --network host -t "$DDDMR_IMAGE" \
    -f "$DDDMR_WS/$DOCKER_DIR/Dockerfile_x64" "$DDDMR_WS/$DOCKER_DIR"
}

build_dddmr_ws() {
  if [[ -n "$(with_docker docker ps -q --filter "name=^${DDDMR_CONTAINER}$")" ]]; then
    die "Container $DDDMR_CONTAINER is running. Stop it before building."
  fi
  log "Building the dddmr workspace in the container..."
  with_docker "$REPO_ROOT/scripts/run_dddmr_container.sh" \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release < /dev/null
}

build_host_interfaces() {
  log "Building dddmr_sys_core on the host in $DDDMR_HOST_WS"
  (
    # Interfaces are generated for the system python, not for .mpc_venv.
    PATH="$(tr ':' '\n' <<< "$PATH" | grep -vF '/.mpc_venv/' | paste -sd:)"
    unset VIRTUAL_ENV
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    set -u
    mkdir -p "$DDDMR_HOST_WS"
    cd "$DDDMR_HOST_WS"
    colcon build --base-paths "$DDDMR_WS/src/dddmr_sys_core" --packages-select dddmr_sys_core
  )
}

main() {
  parse_args "$@"
  check_system
  install_apt_packages
  install_docker

  fetch_dddmr
  apply_patches
  if (( CLEAN )); then
    clean_workspaces
  fi

  build_image
  build_dddmr_ws
  build_host_interfaces
  mkdir -p "$DDDMR_BAGS"

  log "Installation complete."
  log "Container shell: scripts/run_dddmr_container.sh"
  log "Host interfaces for nav_goals.py: source $DDDMR_HOST_WS/install/setup.bash"
  log "Mapping, localization and navigation: navigation/readme.md"
}

main "$@"
