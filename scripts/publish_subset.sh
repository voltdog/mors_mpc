#!/usr/bin/env bash
set -euo pipefail

SRC_REPO="/home/yoggi/mors_mpc"
# TARGET_REPO="git@github.com:yoggi56/quadruped_control.git"
TARGET_REPO="git@github.com:voltdog/mors_quadruped.git"
TARGET_BRANCH="main"   # если нужно, поменяй на master

TMP_DIR="$(mktemp -d)"
trap 'rm -rf "$TMP_DIR"' EXIT

PATHS=(
  common
  config
  lcm_msgs
  LocomotionController
  MorsLogger
  ros_ws/src/mors_keyboard_control
  ros_ws/src/robot_mode_controller
  ros_ws/src/mors_ros_msgs
  Simulator
  pictures
  start_controller.sh
  install.sh
  README.md
  README_rus.md
)

mkdir -p "$TMP_DIR/export"

# Берем только tracked файлы из указанных путей (gitignore учитывается)
mapfile -t FILES < <(git -C "$SRC_REPO" ls-files -- "${PATHS[@]}")

if [ "${#FILES[@]}" -eq 0 ]; then
  echo "Нечего экспортировать: список файлов пуст."
  exit 1
fi

for f in "${FILES[@]}"; do
  mkdir -p "$TMP_DIR/export/$(dirname "$f")"
  cp -a "$SRC_REPO/$f" "$TMP_DIR/export/$f"
done

cd "$TMP_DIR/export"
git init
git checkout -b "$TARGET_BRANCH"
git add .
git commit -m "Subset export from mors_mpc ($(date -u +'%Y-%m-%d %H:%M:%S UTC'))"
git remote add origin "$TARGET_REPO"
git push -f origin "$TARGET_BRANCH"
