#!/bin/bash
set -e  # останавливает выполнение при первой ошибке

# Корневая директория (можно поменять при необходимости)
ROOT_DIR="$(cd -- "$(dirname -- "$0")" && pwd)"

# Массив с именами проектов
PROJECTS=("StateEstimatorLKF" "StateEstimator" "WBIC" "LegController" "MorsLogger" "LocomotionController" "HeightMapBuilder") # "RealsenseCamera" "BHI360_IMU")

# Проверяем, запущено ли с аргументом -c (configure)
WITH_CMAKE=false
if [ "$1" == "-c" ]; then
  WITH_CMAKE=true
fi

# Функция сборки одного проекта
build_project() {
  local project_name=$1
  local project_dir="${ROOT_DIR}/${project_name}"
  local build_dir="${project_dir}/build"

  echo "=============================="
  echo "🔧 Собираю проект: ${project_name}"
  echo "=============================="

  if [ "$WITH_CMAKE" = true ]; then
    mkdir -p "$build_dir"
  fi
  cd "$build_dir"
  
  if [ "$WITH_CMAKE" = true ]; then
    cmake ..
  fi
  make -j

  echo "✅ ${project_name} успешно собран!"
  echo
}

# echo "🚀 Сборка ROS2-пакетов..."
cd "${ROOT_DIR}/ros_ws"
colcon build --packages-select robot_mode_controller mors_experiments #mors_sim 

# Основной цикл
echo " "
echo "🚀 Сборка cmake-пакетов..."
for project in "${PROJECTS[@]}"; do
  build_project "$project"
done
