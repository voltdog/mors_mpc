# ros2lcm_bridge

Мост передаёт команды режимов, переключения кинематической схемы и уставки из ROS 2 в `FSMLocomotion`, а статус и результаты FSM публикует в ROS 2. Имена LCM-каналов обе стороны читают из `$CONFIGPATH/channels.yaml`.

## Сборка

Из корня репозитория в окружении ROS 2 Jazzy. Нужны `lcm-gen`, библиотеки LCM, Eigen, yaml-cpp, vbcontrol, OsqpEigen, Pinocchio и eiquadprog для FSM, а также Python-модули `lcm` и `yaml` для моста. После добавления `params` в `ModeCommand` пересоберите `mors_ros_msgs`, мост и ROS-клиентов. В текстовых вызовах сервиса без `params` это поле равно `0`.

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
export LCM_SERVO_URL='udpm://239.255.76.68:17674?ttl=0'
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
state.body.position[2] = .20
state.joints.position = [0., -1.57, 2.9, 0., 1.57, -2.9,
                        0., 1.57, -2.9, 0., -1.57, 2.9]
state.legs.r1_pos = [.18, -.13, 0.]
state.legs.l1_pos = [.18, .13, 0.]
state.legs.r2_pos = [-.18, -.13, 0.]
state.legs.l2_pos = [-.18, .13, 0.]
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
ros2 topic pub --once /cmd_pose geometry_msgs/msg/Twist '{linear: {z: 0.20}}'
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}}'
ros2 service call /fsm/command mors_ros_msgs/srv/Command '{command: {command: 4}}'
ros2 service call /fsm/command mors_ros_msgs/srv/Command '{command: {command: 5}}'
```

Ожидаемые состояния: `PASSIVE=0 → STANDUP=1 → STANDING=2 → LOCOMOTION=3 → FAILURE=7 → PASSIVE=0`. При `ESTOP` в статусе появятся `fault=true` и причина в `fault_str`. После `CLEAR_FAULT` флаг и описание очищаются. В режиме vision встроенный контроллер публикует SERVO_CMD. В PASSIVE и FAILURE все поля команды обнулены на каждом такте WBIC.

## ROS-интерфейс

| Имя | Тип | Назначение |
|---|---|---|
| `fsm/command` | `mors_ros_msgs/srv/Command` | `SET_MODE=0` с целью `PASSIVE=0`, `STANDING=2` или `LOCOMOTION=3`; `START_ACTION=1` для `CHANGE_KINEMATICS`; `ESTOP=4`, `CLEAR_FAULT=5` |
| `fsm/state` | `mors_ros_msgs/msg/ModeState` | Текущее и целевое состояния, `cur_algorithm`, `cur_action`, `cur_params`, `fault` и `fault_str` |
| `fsm/result` | `mors_ros_msgs/msg/ModeResult` | Подтверждения, конечные результаты и ошибки связи |
| `cmd_vel` | `geometry_msgs/msg/Twist` | Скорость → `ROBOT_CMD` |
| `cmd_vel_stamped` | `geometry_msgs/msg/TwistStamped` | Скорость с проверкой возраста и порядка меток |
| `cmd_pose` | `geometry_msgs/msg/Twist` | Поза → `ROBOT_CMD` |
| `gait_params` | `mors_ros_msgs/msg/GaitParams` | Параметры походки → `GAIT_PARAMS` |

`ACCEPTED=0` в ответе сервиса означает, что FSM приняла запрос. Окончательный результат приходит в `fsm/result`. Если исход неизвестен из-за потери связи или таймаута моста, публикуется `UNKNOWN=255`. После такого сбоя с незавершённым запросом обычные команды требуют перезапуска моста: в LCM-ответах нет идентификатора запроса. При свежем статусе FSM `ESTOP` можно послать во время перехода или после потери подтверждения. `fsm/run_action` пока не предоставляется.

### Переключение кинематической схемы

Для запуска используйте `command=START_ACTION (1)`, `target=CHANGE_KINEMATICS (0)` и поле `params`:

| `params` | Константа | Схема |
|---|---|---|
| 0 | `KNEE_INSIDE` | `><` |
| 1 | `KNEE_OUTSIDE` | `<>` |
| 2 | `KNEE_BACK` | `<<` |
| 3 | `KNEE_FRONT` | `>>` |

Из `STANDING` переключите колени назад:

```bash
ros2 service call /fsm/command mors_ros_msgs/srv/Command \
  '{command: {command: 1, target: 0, params: 2}}'
```

Дождитесь `SUCCEEDED=1` в `/fsm/result` перед следующей обычной командой. Во время выполнения `/fsm/state` содержит `current_state=ACTION (4)`, `cur_action=0` и выбранный `cur_params`; после завершения FSM возвращается в `STANDING`. При уже достигнутой цели действие завершается без движения, и промежуточный статус `ACTION` может не публиковаться.

Перед движением суставов FSM плавно задаёт высоту 0,27 м через `robot_input` и ждёт подтверждения по телеметрии. Высота трактуется как обычная `cmd_pose.linear.z`: мировой Z для `NOADAPT`, высота над опорами для режимов адаптации. Углы для переключения измеряются заново после подготовки.

После переключения FSM возвращает управление в `robot_input` и плавно переходит к актуальной команде высоты. Если команды нет, восстанавливается фактическая высота перед действием. Новая цель, поступившая во время возврата, применяется следующим плавным участком после окончания текущего. `SUCCEEDED` выдаётся после стабилизации на последней цели.

Настройки находятся в `config/locomotion_controller.yaml`, секция `switch_kin_sch`: `preparation_height_m: 0.27`, `max_body_height_speed_mps: 0.05`, `height_motion_min_s: 1.5`. Изменения требуют перезапуска FSM. Ошибка высоты не более 5 мм, вертикальная скорость не более 0,02 м/с и четыре контакта должны сохраняться 0,1 с. На стабилизацию после траектории отводится 2 с; потеря опоры более 0,08 с на высотном этапе приводит к ошибке.

Для сравнения подготовки в MuJoCo без графического окна из корня репозитория:

```bash
.mpc_venv/bin/python FSMLocomotion/tests/compare_switch_height_sim.py \
  FSMLocomotion/build/fsm_locomotion_node --heights 0.20 0.27 --adaptation 2
```

Стенд использует реальные FSM/MPC, модель и суставной PD-контур симулятора; наблюдения поступают непосредственно из MuJoCo. В проверенном прогоне `NOADAPT=2` цикл `<< → <> → <<` с подготовкой 0,27 м завершился без касаний коленями пола, максимальный `pitch` составил около 3,5°. Отдельный прогон `INCL_ADAPT=0` с тем же числом 0,27 завершился по ограничению `abad_R2`: в этом режиме целевой мировой Z выше на высоту опор. Эти результаты не подтверждают устойчивость для всех режимов адаптации и исходных поз.

Мост допускает одну обычную незавершённую команду: повторный запуск и `SET_MODE` во время действия получают `BUSY`. `ESTOP` прерывает действие, результат действия приходит как `FAILED`. При потере связи или превышении тайм-аута мост выдаёт `UNKNOWN` и не повторяет команду. Тайм-аут `result_timeout` по умолчанию равен 30 с; его истечение само по себе не останавливает робота.

Неверные схемы, `GIVE_PAW`, `WAVE_PAW` и `CANCEL_ACTION` отклоняются мостом без отправки LCM. Запуск только из `STANDING`, проверку телеметрии и суставных ограничений выполняет FSM. В остальных командах поле `params` игнорируется. Сервис совместимости `robot_action` сохраняет прежние функции вставания и укладывания.

### Сервисы совместимости

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
