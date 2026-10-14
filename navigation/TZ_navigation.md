# ТЗ: интеграция стека навигации dddmr_navigation с MuJoCo-симулятором mors_mpc

Документ адресован ИИ-агенту-исполнителю. Предполагается, что контекста предыдущих обсуждений у агента нет: все пути, параметры и решения ниже заданы явно. Пошаговый порядок работ приведён в [plan.md](plan.md).

---

## 1. Цель

Обеспечить полный цикл **картирование → локализация → автономная навигация** четырёхногого робота MORS в MuJoCo-симуляторе `mors_mpc` с помощью 3D-стека [dddmr_navigation](https://github.com/dfl-rlab/dddmr_navigation).

Робот оснащается симулированным лидаром **Robosense Airy-96** (решение владельца от 2026-10-12, до этого — Unitree 4D LiDAR L1, см. раздел 2.3). Интерфейсы симулятора (топики, frame_id, поля облака, TF) должны совпадать с теми, что будут на реальном роботе с этим лидаром. Тогда конфигурацию dddmr можно будет перенести на железо без изменений, кроме static TF и калибровки.

## 2. Исходное состояние

### 2.1. mors_mpc (хост, этот репозиторий, `/home/yoggi/mors_mpc`)

- Ubuntu 24.04, **ROS 2 Jazzy** на хосте.
- Запуск симуляции: `./run.sh --sim [--rviz]`.
  - `run.sh` экспортирует `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`.
  - LCM-URL берутся из env: `LCM_DEFAULT_URL`, `LCM_LOCOMOTION_URL`. Путь к конфигам — `CONFIGPATH=<repo>/config`.
- **Симулятор**: `Simulator/mors_simulator.py`, окружение `Simulator/additional/mors_env.py` (класс `MorsMujocoEnv`).
  - Физика 1000 Гц (`config/simulation.yaml: frequency`).
  - Все метки времени — wall clock (`time.time_ns()`), `use_sim_time` нигде не используется.
  - Глубинная камера D435i рендерится в отдельном потоке (`depth_image_loop` в `mors_simulator.py`). Рендер идёт по копии `MjData` (`MorsMujocoEnv.get_depth_image`: копирование `qpos/qvel/...` под `_sim_data_lock` с неблокирующим захватом). **Это образец для потока лидара.**
  - Python-окружение: `.mpc_venv` (Python 3.12, MuJoCo 3.5.0). В текущем `.mpc_venv/pyvenv.cfg` стоит `include-system-site-packages = false`, то есть rclpy в симуляторе недоступен. Связь симулятора с остальной системой идёт только через LCM.
- **MJCF робота**: `Simulator/MJCF/mors.xml`.
  - Тело `base` (freejoint), геом корпуса `body_visual`: box с полуразмерами 0.1059×0.102×0.044.
  - D435i: тело `camera_frame` с pos (0.2491, 0, 0.1201), euler (0 45 90).
  - Геомам робота атрибут `group` не задан, то есть у всех группа 0.
- **Сцены**: `Simulator/MJCF/{flat,ramp,stairs,platform,patch,boxes,boards,stumps}.xml`.
  - Генерируются скриптами `Simulator/MJCF/generate_*.py`; примеры команд — в `Simulator/MJCF/readme.txt`.
  - Выбор сцены: `config/simulation.yaml: scene`.
  - Загрузка: `Simulator/additional/mjcf.py: load_mjmodel(xml_path, scene=...)`. Сцена делает `<include>` модели робота.
- **LCM**: типы в `lcm_msgs/*.lcm` (package `mors_msgs`), генерация `lcm_msgs/lcm_gen.sh`; каналы в `config/channels.yaml`.
- **Мост LCM→ROS**: `ros_ws/src/robot_state_viewer/src/robot_state_viewer_node.cpp`, launch `robot_state_viewer.launch.py`.
  - Публикует `/odom` (nav_msgs/Odometry, twist в системе base_link) и TF `world_frame_id → base_link` по LCM `ROBOT_STATE`. В симуляции это ground truth.
  - `world_frame_id` — launch-аргумент, по умолчанию `world`.
  - Также публикует `/joint_states`, depth, heightmap, маркеры. `robot_state_publisher` поднимается из того же launch.
  - Образец моста облака точек: `pointCloudHandler` (`pointcloud_msg` → PointCloud2).
  - **Сейчас запускается только при `run.sh --rviz`.**
- **Управление**: `ros_ws/src/robot_mode_controller`.
  - Подписки: `cmd_vel` (Twist), `cmd_pose` (Twist), `gait_params` (`mors_ros_msgs/msg/GaitParams`).
  - Сервисы: `robot_mode` и `robot_action` (`mors_ros_msgs/srv/RobotCmd`, поле `int8 data`).
  - **Таймаута по `cmd_vel` нет**: последняя команда держится бесконечно.
- **Образец автоматического подъёма робота**: `ros_ws/src/mors_experiments_sim/mors_experiments_sim/exp2a_curvilinear.py`.
  - `robot_action`: `STANDUP=1`, `LAY_DOWN=2`; `robot_mode`: `LOCOMOTION_MODE=1`, `DO_NOTHING_MODE=0`.
  - Подъём длится ~2.5–3 с без обратной связи о завершении.
- **Клавиатура**: `ros_ws/src/mors_keyboard_control`.
  - Публикует `cmd_vel`, `cmd_pose`, `gait_params` только при изменениях.
  - Типовые значения: высота корпуса 0.206 м, `t_sw` 0.18 с, `t_st` 0.5…0.2 с в зависимости от максимальной скорости 0.1…0.9.

### 2.2. dddmr_navigation (`~/dddmr_navigation`)

- Сейчас собран образ `dddmr:humble`. **По решению владельца проекта стек пересобирается под Jazzy**: в репозитории есть `dddmr_docker/docker_file_2404/{Dockerfile_x64,build.bash,run_x64.bash}`, образ `dddmr:jazzy`.
- Требования стека к роботу (`src/dddmr_beginner_guide/README.md`):
  - вход `/cmd_vel` (Twist);
  - выходы робота: облако лидара (PointCloud2), `/odom` (Odometry), TF `odom → base_link → <lidar_frame>`;
  - static TF `base_link → base_footprint` (используется как `base_ground_frame`).
- Полезные образцы конфигурации:
  - `src/dddmr_beginner_guide/launch/airy_tilt45_{mapping,navigation}.launch` и `config/airy_tilt45_*.yaml` — купольный лидар с наклоном 45° вниз на четвероногом роботе. **Ближайший аналог нашей установки.**
  - `src/dddmr_p2p_move_base/launch/go2_{mapping,localization}.launch`, `config/go2_*.yaml` — Go2 в Gazebo. В них `use_sim=true`; нам нужно `false`.
  - `src/dddmr_perception_3d/config/multilayer_spinning_lidar_demo/scanning_lidar_3d_ros.yaml` — конфиг perception_3d для Unitree L1 (образец параметров слоя `MultiLayerSpinningLidar`): `topic: unilidar/cloud`, `vertical_FOV_top: 90`, `vertical_FOV_bottom: 0`, `stitcher_num: 8`.
  - `src/dddmr_lego_loam/lego_loam_bor/config/loam_bag_mid360_config.yaml` — пример для неповторяющегося (non-repetitive) лидара.
- **Важное ограничение LeGO-LOAM** (`src/dddmr_lego_loam/lego_loam_bor/src/imageProjection.cpp`, ~стр. 428): облако считается невалидным, если в нём меньше `0.1 · num_vertical_scans · num_horizontal_scans · stitcher_num` точек. Для Airy (86 400 точек в кадре, без склейки) это ограничение выполняется с запасом при 96 × 900. Номер кольца LeGO-LOAM вычисляет из угла места точки, поле `ring` ему не нужно.

### 2.3. Robosense Airy-96 (паспорт и драйвер)

- **Замена L1 на Airy (решение владельца, 2026-10-12).** С L1 (2160 точек в кадре) lego_loam уводил высоту, и карта здания получалась с ошибкой 16–28 см. Тот же маршрут с облаком Airy, пересчитанным в MuJoCo, дал 5.9 см без дублирования этажей (navigation/plan.md, этап 6). Код и конфиг L1 удалены, архив — `navigation/archive/l1_2026-10-12.tar.gz`.
- Паспорт: полусфера 360° × 90°, 96 каналов, 10 Гц, ~860 тыс. точек/с, то есть 900 точек на канал (0.4°) и 86 400 в кадре. Дальность до 60 м (30 м при отражении 10 %). Корпус Ø60 × 63 мм, < 240 г.
- ROS 2 драйвер [rslidar_sdk](https://github.com/RoboSense-LiDAR/rslidar_sdk):
  - облако `/rslidar_points`, `frame_id: rslidar` (`config/config.yaml`);
  - тип точки XYZIRT: `x, y, z, intensity` — FLOAT32, `ring` — UINT16, `timestamp` — FLOAT64. Поля идут подряд без выравнивания (`addPointField`): смещения 0/4/8/12/16/18, `point_step 26`.
- Записи с реального Airy нет. Начало СК, нулевой азимут и углы каналов в модели — приближение (см. 4.2).

### 2.4. MuJoCo-LiDAR

- Репозиторий [TATP-233/MuJoCo-LiDAR](https://github.com/TATP-233/MuJoCo-LiDAR), PyPI `mujoco-lidar`, MIT, Python 3.10–3.13, `mujoco>=3.2`.
- API (`docs/en/API.md`):
  - `MjLidarWrapper(mj_model, site_name, backend="cpu", cutoff_dist=..., args={"geomgroup": ndarray|None, "bodyexclude": int})`.
  - `trace_rays(mj_data, theta, phi)` → дальности. `theta` — азимут, `phi` — угол места, в радианах, в системе site.
  - `get_hit_points()` → точки в системе site. **Промахи имеют dist = −1 и точку (0,0,0), их надо отбрасывать.**
- CPU-бэкенд — обёртка над `mujoco.mj_multiRay` (~9 M лучей/с), GPU не нужен.
- `bodyexclude` исключает только **одно** тело, поэтому самоисключение робота делается через `geomgroup`.
- Готовые шаблоны: HDL64, VLP32, OS128, Airy96 (`generate_airy96`: 96 × 900, угол места 0…89.5°), Livox (mid360, avia и др.) и сетка.

## 3. Целевая архитектура

```
 ┌──────────── хост (ROS 2 Jazzy) ─────────────────────────────────────────────┐
 │ MuJoCo (mors_simulator.py)                                                  │
 │   ├─ ROBOT_STATE (LCM) ─┐                                                   │
 │   └─ LIDAR (LCM, новый)─┤                                                   │
 │                         ▼                                                   │
 │   robot_state_viewer ── /odom, TF odom→base_link, /rslidar_points           │
 │   static TF: base_link→rslidar, base_link→base_footprint                    │
 │                                                                             │
 │   /cmd_vel ──► robot_mode_controller ◄── mors_keyboard_control              │
 │                (подъём, режим, пресет скорости — вручную с клавиатуры)      │
 └─────────────────────────────────────────────────────────────────────────────┘
            ▲ DDS (--network=host, один ROS_DOMAIN_ID, UDPv4)
 ┌──────────┴─ docker dddmr:jazzy ─────────────────────────────────────────────┐
 │ lego_loam (картирование, map→odom) | mcl_3dl (локализация, map→odom)        │
 │ pg_map_server, perception_3d, global_planner, local_planner, p2p_move_base  │
 │   → /cmd_vel                                                                │
 └─────────────────────────────────────────────────────────────────────────────┘
```

Дерево TF: `map → odom → base_link → {rslidar, base_footprint, звенья робота из URDF}`. `map → odom` публикует только dddmr: lego_loam в режиме картирования, mcl_3dl в режиме локализации.

## 4. Требования по компонентам

### 4.1. Docker-образ dddmr под Jazzy

1. Собрать образ `dddmr:jazzy` скриптом `~/dddmr_navigation/dddmr_docker/docker_file_2404/build.bash` (тип `x64`, без CUDA).
2. Собрать workspace dddmr внутри контейнера: `colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release`. Сборочные артефакты Humble (`build/ install/ log/` в `~/dddmr_navigation`) несовместимы с Jazzy. Перед сборкой их нужно перенести в резервную копию или удалить. **Спросить владельца, прежде чем удалять.**
3. Написать свой скрипт запуска контейнера в репозитории mors_mpc (например, `scripts/run_dddmr_container.sh`) на основе `docker_file_2404/run_x64.bash`:
   - `--network=host`, `--privileged`, проброс `DISPLAY` для rviz2;
   - `ROS_DOMAIN_ID`, `RMW_IMPLEMENTATION` (если задан) и `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` — такие же, как на хосте;
   - volume'ы: `~/dddmr_navigation:/root/dddmr_navigation`, `~/dddmr_bags:/root/dddmr_bags`, пакет `mors_dddmr` (см. 4.6) в `/root/dddmr_navigation/src/mors_dddmr`.
4. **Исходники `~/dddmr_navigation` не модифицировать.** Вся настройка под MORS живёт в пакете `mors_dddmr`.

### 4.2. Модель Robosense Airy в симуляторе

1. `mujoco-lidar` в `.mpc_venv`, строка в `install.sh` (функция `setup_python_venv`). Бэкенд `cpu`.
2. Шаблон кадра — модуль `Simulator/additional/robosense_airy.py` (`RobosenseAiryScanPattern`): неизменная сетка `channels × horizontal_points` (96 × 900), угол места `elevation_range_deg` (0…89.5°) равномерно, азимут 0…360° с шагом 0.4°. Порядок точек — по столбцам (азимут снаружи, каналы внутри), `ring` — номер канала. Частота кадров `frame_rate` = 10 Гц. Все величины — параметры конфига. Равномерная сетка углов места, начало СК и нулевой азимут — **приближение**, их нужно сверить с документацией и записью реального Airy.
3. Трассировка лучей идёт в отдельном потоке `lidar_loop`, по образцу `depth_image_loop`:
   - снимок `qpos` под `_sim_data_lock` (захват с таймаутом 5 мс), затем `mujoco.mj_kinematics` на копии, затем `MjLidarWrapper.trace_rays`;
   - кадр мгновенный: вся геометрия с одного снимка, времени точек внутри кадра нет, компенсация движения (deskew) не нужна;
   - кадр считается ~50 мс. GIL при этом почти отпущен, а сериализация LCM сделана на numpy (`encode_lidar_msg`), чтобы **цикл физики 1000 Гц не проседал** (критерий 4).
4. Отбрасывать промахи и точки вне `[range_min, range_max] = [0.1, 60]` м. Опционально гауссов шум по дальности (`range_noise_std`, по умолчанию 0). `intensity` — константа 100.
5. Самоисключение робота: геомы робота в группе 1 (`<default class="mors">`), лучи пускаются по маске `geomgroup`. Флаг `see_self` (сейчас `False`): если включён, ноги и корпус видны лидару, как на реальном железе. Корпус лидара — в группе 2, лучам он не виден никогда: лучи выходят изнутри корпуса.
6. Включение лидара — `lidar: True` в `config/simulation.yaml`. При `False` масса корпуса обнуляется, корпус скрывается, поведение симулятора не меняется.

### 4.3. Установка лидара на робот

1. Лидар стоит **спереди на корпусе, купол наклонён вперёд-вниз**, pitch 45°, как в DDDMR-примере `airy_tilt45_*`. **Временное значение (решение владельца, до задания реального кронштейна):** `xyz = (0.18, 0, 0.16)`, `rpy = (0, 45°, 0)`.
2. **Единый источник истины** — секция `robosense_airy` в `config/sensors.yaml`: `mount.xyz`, `mount.rpy_deg` относительно `base`, параметры шаблона из 4.2, `range_min`, `range_max`, `range_noise_std`, `intensity`, `see_self`, `frame_id: rslidar`.
3. В `mors.xml` в теле `base`: тело `robosense_airy` с site и визуальным корпусом (цилиндр Ø60 × 63 мм, `contype="0" conaffinity="0"`, масса 0.24 кг). Симулятор при загрузке выставляет pose тела из `sensors.yaml`.
4. Static TF `base_link → rslidar` строится из тех же значений `sensors.yaml`, вручную числа не дублировать.

### 4.4. Транспорт облака: LCM → ROS 2

1. LCM-тип `lcm_msgs/lidar_msg.lcm` (package `mors_msgs`): `int64_t timestamp` — момент снимка состояния, Unix ns; `int32_t points_count`; массивы `float x[], y[], z[], intensity[]`, `int16_t ring[]`. Координаты — в системе лидара. Кадр ~1.55 МБ, канал `lidar: "LIDAR"` в `config/channels.yaml`.
2. В `robot_state_viewer` обработчик `LIDAR → sensor_msgs/PointCloud2` (`lidarHandler`):
   - топик `/rslidar_points`, `frame_id: rslidar` (параметры узла `lidar_ros_topic`, `lidar_frame_id`);
   - поля и раскладка как у `rslidar_sdk` XYZIRT (2.3), `is_dense=true`, `height=1`;
   - `header.stamp` = `timestamp`, `timestamp` каждой точки — тот же момент в секундах. У реального драйвера время точек растянуто на оборот 0.1 с, а в заголовке стоит время первой точки. В симуляции весь кадр снят в один момент, и штамп должен совпадать с моментом снимка, чтобы TF искался на правильное время.

### 4.5. Одометрия и TF

1. В режиме навигации `robot_state_viewer` запускается с `world_frame_id:=odom`. Тогда TF `odom → base_link` и `/odom` (`header.frame_id = odom`) берутся из ground truth симулятора. Карта высот и прочие облака в этом режиме тоже будут в `odom`, это допустимо.
2. Static TF:
   - `base_link → base_footprint`: (0, 0, −0.206) — номинальная высота корпуса, как `KEYBOARD_BODY_Z` в `mors_keyboard_control.py`;
   - `base_link → rslidar` — из `sensors.yaml` (см. 4.3).
3. `use_sim_time = false` во всех узлах dddmr. Хост и контейнер работают на одних часах.

### 4.6. ROS-пакет интеграции `mors_dddmr` и управление роботом

**`mors_dddmr`** (`mors_mpc/ros_ws/src/mors_dddmr`) — ament_cmake, только `launch/`, `config/`, `rviz/`. Монтируется в контейнер dddmr и собирается там. На хосте собирается без зависимостей от dddmr: в `package.xml` пакеты dddmr только как `exec_depend`.

- `launch/mors_airy_mapping.launch` — по образцу `airy_tilt45_mapping.launch` / `go2_mapping.launch`:
  - lego_loam, global_planner, p2p_move_base, clicked2goal, rviz2;
  - remap входного облака на `/rslidar_points`; выход `cmd_vel` остаётся `/cmd_vel` без remap;
  - `use_sim_time: false`.
- `launch/mors_airy_localization.launch` — по образцу `go2_localization.launch`:
  - mcl_feature, pg_map_server, mcl_3dl, global_planner, p2p_move_base, clicked2goal, rviz2;
  - те же remap.
- `config/mors_airy_mapping.yaml`, `config/mors_airy_localization.yaml`. Отправная точка — `go2_*.yaml` и `airy_tilt45_*.yaml`. Обязательно настроить:
  - **lego_loam `laser.*`**: `vertical_angle_bottom: 0`, `vertical_angle_top: 89.5`, `num_vertical_scans × num_horizontal_scans` = 96 × 900, `scan_period: 0.1`, `odom_type: "wheel_odometry"` (ground-truth odom как prior), `base_ground_frame: "base_link"`. С `base_footprint` позы карты и `map→odom` сдвигаются на 0.206 м, см. plan.md, этап 6;
  - **`imageProjection.stitcher_num: 1`** (без склейки). Также `ground_fov_*` под наклон 45° и `minimum_detection_range` с учётом корпуса;
  - **`mapping.corner_voxel_size`, `mapping.surface_voxel_size`: 0.1** (параметры из патча `navigation/patches/lego_loam_bor.patch`). С зашитыми в lego_loam 0.2 / 0.4 м scan-to-map занижает перемещение на ~2 %, см. plan.md, этап 6a;
  - **perception_3d**: плагин `MultiLayerSpinningLidar` на облаке `segmented_cloud_pure` из mcl_feature (земля, пандус и ступени уже удалены; так во всех конфигах dddmr — решение владельца 2026-10-13), `stitcher_num: 1`. Полоса отметки `marking_minimum_height`…`marking_height` задаётся по z в base_link: пол на −0.206, низ антресоли на +0.334. Глобальный слой отмечает только сектор земли mcl_ip (азимут ±40°, угол места 0…35°), см. plan.md, этап 8;
  - **размер робота**: у глобального планировщика это только `inscribed_radius`, у локального — кубоид в base_link (габарит геомов с запасом). `robot_radius` траекторного генератора влияет лишь на ограничение оборотов колёс (выключено);
  - **локальный планировщик**: `max_vel_x` 0.4, `min_vel_x` 0.1, `max_vel_theta` 0.6 (стартовые значения), ускорения — под возможности MPC-контроллера MORS;
  - **pg_map_server**: `pose_graph_dir: /root/dddmr_bags/mors_building`.
- `rviz/` — конфиги на основе rviz-конфигов dddmr, с облаком `/rslidar_points` и моделью робота.

**Управление роботом.** Отдельного моста нет: `p2p_move_base` публикует прямо в `/cmd_vel`, который слушает `robot_mode_controller`. Перед картированием и навигацией робота готовят с клавиатуры (`ros2 run mors_keyboard_control mors_keyboard_control`):
1. `Enter` — подъём.
2. `Space` — `LOCOMOTION_MODE`. Клавиатура публикует `gait_params` со `standing=False`, без этого робот не шагает.
3. Клавиша `1..9` — пресет скорости. Он должен быть не ниже `max_vel_x` навигации, потому что от него зависит `t_st` в `gait_params`.

Клавиатура публикует `cmd_vel` только при изменении, поэтому с dddmr она не конфликтует: нажатие клавиши перехватывает управление до следующей команды стека. После достижения цели `p2p_move_base` публикует нулевую скорость (`publishZeroVelocity`). Робот при этом шагает на месте, пока его не переведут в стойку клавишей `Space`.

### 4.7. Запуск

1. Новый флаг `run.sh --nav`:
   - поднимает `robot_state_viewer` с `world_frame_id:=odom` (без rviz; при `--rviz` — с ним);
   - устанавливает или проверяет `lidar: True`.
   Без флага поведение `run.sh` не меняется.
2. Скрипт запуска контейнера (4.1) и launch'и `mors_dddmr` для режимов картирования и локализации/навигации.
3. В README (рус. и англ.: `README.md`, `README_rus.md`) — раздел «Навигация (dddmr)»: сборка образа, запуск симуляции с `--nav`, картирование с сохранением карты, локализация и задание начальной позы, отправка целей.

### 4.8. Тестовый мир `building` (многоуровневый)

1. Генератор `Simulator/MJCF/generate_building.py` в стиле существующих `generate_*.py`: аргументы командной строки, шапка с аргументами в сгенерированном XML, `<include file="{path}"/>`. Результат — `Simulator/MJCF/building.xml`. Добавить `"building"` в список сцен в `config/simulation.yaml` (комментарий) и команду в `Simulator/MJCF/readme.txt`.
2. Состав сцены:
   - **первый этаж** ≈12×8 м: несколько комнат и коридор, дверные проёмы ≥0.8 м, стены высотой 1.5–2 м, несколько ящиков-препятствий высотой 0.2–0.5 м;
   - **пандус** с уклоном ≤12–15° (проверить проходимость контроллером; есть готовая сцена `ramp` 15°) на **антресоль** высотой ≈0.6 м, с бортиком или перилами по открытым краям;
   - **под антресолью робот может проходить** (просвет ≥0.45 м, опоры-колонны). Это перекрывающиеся уровни — ключевой сценарий dddmr;
   - достаточно вертикальных структур и неоднородностей для scan-matching (без длинных однообразных коридоров без признаков).
3. Начальная поза робота (`init_body_pos`) — в свободной зоне первого этажа.

## 5. Ограничения и правила

- Соблюдать `CLAUDE.md`: чистый, лаконичный, идиоматичный код в стиле окружающего. Существующий код без сильной необходимости не менять; изменения в `robot_state_viewer`, `mors.xml`, `mors_simulator.py`, `mors_env.py`, `run.sh`, `install.sh` — только аддитивные.
- Не ломать `./run.sh --sim`, `./run.sh --sim --rviz` и сценарии `experiments_sim/*.sh`.
- Перед написанием алгоритмов (генератор шаблона, метрики карты) искать готовые реализации и публикации.
- Не модифицировать `~/dddmr_navigation/src`. Исключение, согласованное с владельцем: `navigation/patches/lego_loam_bor.patch` (размеры вокселей scan-to-map как параметры, исправление поз в `poses.pcd`), применяется `git apply` в `~/dddmr_navigation`. Не удалять данные (`~/dddmr_navigation/{build,install,log}`, `~/dddmr_bags`) без подтверждения владельца.
- Не коммитить без явной просьбы владельца.
- При неоднозначности или противоречии в ТЗ задавать вопрос владельцу, а не выбирать молча.

## 6. Критерии приёмки

| # | Критерий | Как проверить |
|---|---|---|
| 1 | `/rslidar_points` публикуется с частотой 10 ± 1 Гц, 86 400 ± 10 % точек в кадре в замкнутом помещении | `ros2 topic hz`, подсчёт `width` |
| 2 | Поля и типы облака как у `rslidar_sdk` XYZIRT: `x,y,z,intensity` FLOAT32, `ring` UINT16, `timestamp` FLOAT64, `point_step 26`; `frame_id: rslidar` | `ros2 topic echo --once --field fields` |
| 3 | Без шума точки лежат на геометрии сцены: медианная ошибка < 1 см | Сравнение с `mj_ray` / аналитической геометрией в тестовом скрипте |
| 4 | Включение лидара снижает фактическую частоту цикла физики не более чем на 5 % | Замер периода цикла `mors_simulator.py` с лидаром и без |
| 5 | Дерево TF: `map→odom→base_link→{rslidar, base_footprint}`, без конфликтов и дублей | `ros2 run tf2_tools view_frames` в контейнере |
| 6 | Контейнер `dddmr:jazzy` видит топики хоста и наоборот | `ros2 topic list/hz` с обеих сторон |
| 7 | Картирование: проход с клавиатуры по всему зданию, включая антресоль и проход под ней; карта сохранена в `~/dddmr_bags/mors_building` | Файлы pose graph на диске |
| 8 | Карта геометрически верна: средняя дистанция точек карты до поверхностей сцены < 5 см после выравнивания карты по стартовой позе; нет дублирования этажей | Скрипт сравнения карты с геометрией MJCF |
| 9 | Локализация: после задания начальной позы ошибка < 10 см (3D) и < 3° (рыскание) на всём тестовом маршруте. Зачётная величина — p95 после окна сходимости 10 с. Ошибка считается относительно карты: истинная поза в `map` получается через ближайший ключевой кадр, поэтому собственная ошибка карты в неё не входит. Если z не сходится из-за привязки частиц mcl_3dl к полу, порог положения проверяется по xy, а z приводится для справки (`map_check.py loc --xy`) (решения владельца, 2026-10-12) | Лог `map→odom` (`traj_log.py`) с GT `odom→base_link` против позы из симулятора: `map_check.py loc`, ошибка против GT выводится для справки |
| 10 | Навигация: из 5 целей (на антресоли, под антресолью, за дверным проёмом, в дальней комнате, возврат на старт; координаты — `nav_goals.py`, plan.md, этап 8) достигнуто ≥ 4; корпус не касается стен | Скрипт отправки целей + лог контактов MuJoCo |
| 11 | При достижении и при отмене цели робот останавливается: последняя команда в `/cmd_vel` от dddmr нулевая | `ros2 topic echo /cmd_vel` во время теста навигации |
| 12 | Регрессия: `run.sh --sim` без `--nav` и `experiments_sim` работают как раньше | Прогон `experiments_sim/exp2a.sh` |

## 7. Результаты работы

1. Код и конфиги: модель Airy, поток лидара, LCM-тип, мост в `robot_state_viewer`, пакет `mors_dddmr`, флаг `--nav`, скрипт контейнера, изменения в `install.sh`.
2. Генератор и файл сцены `building`.
3. Сохранённая карта `~/dddmr_bags/mors_building`.
4. Разделы README (рус./англ.).
5. Скрипты проверки критериев 1–4 и 8–11.
6. Короткий отчёт `navigation_report.md`: значения метрик по критериям, выбранные параметры (V×H, `stitcher_num`, скорости, монтаж лидара), известные ограничения.

## 8. Ссылки

- dddmr_navigation: [README](https://github.com/dfl-rlab/dddmr_navigation), [beginner guide](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_beginner_guide), [lego_loam](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_lego_loam), [mcl_3dl](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_mcl_3dl), [perception_3d](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_perception_3d), [p2p_move_base](https://github.com/dfl-rlab/dddmr_navigation/tree/main/src/dddmr_p2p_move_base). Локальная копия: `~/dddmr_navigation`.
- MuJoCo-LiDAR: [репозиторий](https://github.com/TATP-233/MuJoCo-LiDAR), `docs/en/API.md`, `docs/en/USAGE.md`, пример `examples/unitree_go2_ros2.py`.
- Robosense Airy: [rslidar_sdk](https://github.com/RoboSense-LiDAR/rslidar_sdk) (`config/config.yaml`, `src/source/source_pointcloud_ros.hpp`), [страница Airy](https://www.robosense.ai/en/rslidar/Airy).
- MuJoCo: `mj_multiRay`, `mj_kinematics` — документация MuJoCo 3.x.

## 9. Риски и открытые вопросы

- **Нагрузка CPU и LCM от Airy.** Кадр — 86 400 лучей (~50 мс на ядре) и ~1.55 МБ в LCM на 10 Гц. Цикл физики не просел (критерий 4), но запаса немного: при росте нагрузки уменьшать `horizontal_points` или `frame_rate`. Для LCM нужен `net.core.rmem_max` ≥ 2 МБ (сейчас 2.6 МБ).
- **Дрейф lego_loam по высоте.** Даже с Airy lego_loam уточняет позу только по сканам, одометрия служит лишь начальным приближением. Если критерий 8 недостижим, зафиксировать в отчёте достигнутые значения и предложить варианты, а не подгонять тест.
- **Число точек в кадре зависит от сцены.** Лучи в небо промахиваются. Критерий 1 (86 400 ± 10 %) выполним только в замкнутом помещении с потолком (`building`).
- **Приближённость модели Airy.** Углы каналов, начало СК и нулевой азимут взяты по паспорту и MuJoCo-LiDAR. Их нужно уточнить по документации и записи реального Airy.
- **Фактор реального времени.** Если симулятор не держит 1000 Гц, метки времени (wall clock) остаются согласованными, но динамика замедляется. Оценить в отчёте, как это влияет на локализацию.
- **Нет watchdog по `cmd_vel`.** `robot_mode_controller` держит последнюю команду бесконечно. Если контейнер dddmr упадёт или пропадёт DDS-связь во время движения, робот продолжит идти. В симуляции риск принят владельцем; перед переносом на реальный робот к нему нужно вернуться.
- **Совместимость Jazzy-образа dddmr.** Beginner guide dddmr описывает только Humble-образ, а Jazzy-образ (`docker_file_2404`) в документации не описан. При ошибках сборки сначала зафиксировать их и спросить владельца, прежде чем патчить исходники dddmr.
