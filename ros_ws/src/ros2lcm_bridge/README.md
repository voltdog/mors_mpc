# ros2lcm_bridge

Мост передаёт команды режимов и уставки из ROS 2 в `FSMLocomotion`, а статус и результаты FSM публикует в ROS 2. Имена LCM-каналов обе стороны читают из `$CONFIGPATH/channels.yaml`.

## Сборка

Из корня репозитория в окружении ROS 2 Jazzy. Нужны `lcm-gen`, библиотеки LCM, Eigen и yaml-cpp для FSM, а также Python-модули `lcm` и `yaml` для моста. После изменения `ModeState` пересоберите мост и ROS-клиентов, которые используют это сообщение.

```bash
source /opt/ros/jazzy/setup.bash
cmake -S FSMLocomotion -B /tmp/mors-fsm-build -DPython3_EXECUTABLE=/usr/bin/python3
cmake --build /tmp/mors-fsm-build -j2
colcon build --base-paths ros_ws/src --packages-up-to ros2lcm_bridge \
  --build-base /tmp/mors-bridge-build --install-base /tmp/mors-bridge-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 \
  -DFSM_TEST_BINARY=/tmp/mors-fsm-build/fsm_locomotion_node
```

## Ручная проверка

В терминалах, где запускаются FSM, мост и источник тестовой телеметрии, задайте из корня репозитория:

```bash
export CONFIGPATH="$PWD/config"
export LCM_LOCOMOTION_URL='udpm://239.255.76.67:17673?ttl=0'
```

Используйте отдельный LCM-порт для стенда. При использовании старых ROS-сервисов запустите мост отдельно от `robot_mode_controller`, который регистрирует те же сервисы.

Запустите FSM и мост в разных терминалах:

```bash
# Терминал FSM
/tmp/mors-fsm-build/fsm_locomotion_node
```

```bash
# Терминал моста
source /opt/ros/jazzy/setup.bash
source /tmp/mors-bridge-install/setup.bash
ros2 launch ros2lcm_bridge bridge.launch.py
```

Обе стороны берут URL из `LCM_LOCOMOTION_URL`, затем из `LCM_DEFAULT_URL`, если первая переменная пуста. Параметр `lcm_url:=...` в launch явно задаёт адрес моста.

Для переходов из `PASSIVE` и сброса аварии FSM нужен поток свежих пакетов `ROBOT_STATE` с обязательными флагами готовности. Возьмите поток от оценщика на том же LCM-адресе или подайте синтетическое состояние в отдельном терминале на изолированном порту:

```bash
bindings=$(mktemp -d)
lcm-gen -p --ppath "$bindings" lcm_msgs/robot_state_msg.lcm
PYTHONPATH="$bindings" /usr/bin/python3 - <<'PY'
import os
from pathlib import Path
import time

import lcm
import yaml
from mors_msgs import robot_state_msg

channels = yaml.safe_load((Path(os.environ['CONFIGPATH']) / 'channels.yaml').read_text())
bus = lcm.LCM(os.environ['LCM_LOCOMOTION_URL'])
state = robot_state_msg()
state.body.orientation_quaternion = [0., 0., 0., 1.]
state.legs.contact_states = [True] * 4
state.readiness = [True] * 18 + [False]
try:
    while True:
        state.timestamp = time.time_ns()
        bus.publish(channels['robot_state'], state.encode())
        time.sleep(.005)
except KeyboardInterrupt:
    pass
PY
```

В терминале ROS-клиента подключите то же окружение ROS 2 и overlay моста. Запустите `ros2 topic echo /fsm/state` и `ros2 topic echo /fsm/result` в отдельных терминалах. Перед следующей командой режима дождитесь `SUCCEEDED=1` в `/fsm/result`:

```bash
ros2 service call /fsm/command mors_ros_msgs/srv/Command '{command: {command: 0, target: 2}}'
ros2 service call /fsm/command mors_ros_msgs/srv/Command '{command: {command: 0, target: 3}}'
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}'
ros2 service call /fsm/command mors_ros_msgs/srv/Command '{command: {command: 4}}'
ros2 service call /fsm/command mors_ros_msgs/srv/Command '{command: {command: 5}}'
```

Ожидаемые состояния: `PASSIVE=0 → STANDUP=1 → STANDING=2 → LOCOMOTION=3 → FAILURE=7 → PASSIVE=0`. При `ESTOP` в статусе появятся `fault=true` и причина в `fault_str`. После `CLEAR_FAULT` флаг и описание очищаются. Физические выходы текущей FSM работают как заглушки.

## ROS-интерфейс

| Имя | Тип | Назначение |
|---|---|---|
| `fsm/command` | `mors_ros_msgs/srv/Command` | `SET_MODE=0` с целью `PASSIVE=0`, `STANDING=2` или `LOCOMOTION=3`, `ESTOP=4`, `CLEAR_FAULT=5` |
| `fsm/state` | `mors_ros_msgs/msg/ModeState` | Текущее и целевое состояния, `cur_algorithm`, `cur_action`, `cur_params`, `fault` и `fault_str` |
| `fsm/result` | `mors_ros_msgs/msg/ModeResult` | Подтверждения, конечные результаты и ошибки связи |
| `cmd_vel` | `geometry_msgs/msg/Twist` | Скорость → `ROBOT_CMD` |
| `cmd_vel_stamped` | `geometry_msgs/msg/TwistStamped` | Скорость с проверкой возраста и порядка меток |
| `cmd_pose` | `geometry_msgs/msg/Twist` | Поза → `ROBOT_CMD` |
| `gait_params` | `mors_ros_msgs/msg/GaitParams` | Параметры походки → `GAIT_PARAMS` |

`ACCEPTED=0` в ответе сервиса означает, что FSM приняла запрос. Окончательный результат приходит в `fsm/result`. Если исход неизвестен из-за потери связи или таймаута моста, публикуется `UNKNOWN=255`. После такого сбоя с незавершённым запросом обычные команды требуют перезапуска моста: в LCM-ответах нет идентификатора запроса. При свежем статусе FSM `ESTOP` можно послать во время перехода или после потери подтверждения. `fsm/run_action` пока не предоставляется.

При `legacy_services=true` доступны сервисы `mors_ros_msgs/srv/RobotCmd`:

| Сервис | `data` | Цель `SET_MODE` |
|---|---|---|
| `robot_mode` | 0 | `PASSIVE` |
| `robot_mode` | 1 | `LOCOMOTION` |
| `robot_mode` | 2, 3 | `STANDING` |
| `robot_action` | 1 | `STANDING` |
| `robot_action` | 2 | `PASSIVE` |

Сервис совместимости возвращает `success=1` после принятия или завершения, `0` при отказе, `-1` при неизвестном результате.

`SET_MODE(LOCOMOTION)` разрешает автостарт. Итоговый `SUCCEEDED` подтверждает разрешение в стойке или при ходьбе. Ходьба начинается при свежей скорости выше порога старта и завершается после выдержки ниже порогов остановки и завершения походки. Явный `STANDING` запрещает автостарт и сохраняет возможность восстановления после толчка. Статус `LOCOMOTION` используется и при ходьбе, и при восстановлении.

Каждое допустимое сообщение скорости или позы отправляет полный `ROBOT_CMD`. Если последняя скорость ROS старше `velocity_timeout` моста (по умолчанию 0,5 с), обновление позы отправляется с нулевой скоростью. FSM измеряет свежесть от приёма последнего `ROBOT_CMD`, включая пакет позы со скопированной скоростью. Если пакетов больше нет, FSM обнуляет скорость по `fsm_locomotion.command_timeout_s` (по умолчанию 0,5 с) и начинает отсчёт выдержки автоостановки.

## Автоматическая проверка

```bash
ctest --test-dir /tmp/mors-fsm-build --output-on-failure
colcon test --build-base /tmp/mors-bridge-build --install-base /tmp/mors-bridge-install \
  --packages-select ros2lcm_bridge --event-handlers console_direct+
colcon test-result --test-result-base /tmp/mors-bridge-build/ros2lcm_bridge
```

С `FSM_TEST_BINARY` сквозной тест запускает отдельно FSM и мост, подаёт тестовый `ROBOT_STATE` по LCM и проверяет статусы, переходы, отказ неподдерживаемых команд, сброс аварии и передачу уставок.
