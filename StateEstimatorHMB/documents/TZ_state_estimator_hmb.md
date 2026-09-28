# Техническое задание на модуль `StateEstimatorHMB`

## Назначение

`StateEstimatorHMB` объединяет в одном C++20-процессе функции модулей `RealsenseCamera`, `RealsenseCameraD435i`, `StateEstimator` и `HeightMapBuilder`.

Цель объединения - убрать внутренние задержки и рассинхрон, возникающие при передаче промежуточных `ODOMETRY`, `DEPTH_IMAGE` и `ROBOT_STATE` через LCM между отдельными процессами. LCM должен использоваться только на внешней границе модуля: для входных датчиков робота и публикации результатов.

## Входы и выходы

Входные LCM-каналы берутся из `config/channels.yaml`:

- `IMU_DATA`;
- `SERVO_STATE`;
- `CONTACT_SENSOR`.

Внутренние источники RealSense:

- T265 pose stream, serial из `config/realsense_camera.yaml`;
- D435i depth stream, параметры из `config/realsense_camera_d435i.yaml`.

Выходные LCM-каналы:

- `ROBOT_STATE` с типом `mors_msgs::robot_state_msg`;
- `ROBOT_STATE_CHECK` с тем же типом — полноценное состояние робота от второго
  оценщика, для сравнения двух оценок в одном логе;
- `HEIGHTMAP` с типом `mors_msgs::heightmap_msg`;
- опционально `POINCLOUD` с типом `mors_msgs::pointcloud_msg`;
- при необходимости `SERVO_STATE_FILTERED`.

Промежуточные `ODOMETRY` и `DEPTH_IMAGE` внутри нового контура не публикуются.

## Конфигурация

Модуль обязан использовать существующие конфиги без добавления обязательного нового yaml-файла:

- `config/channels.yaml`;
- `config/timesteps.yaml`;
- `config/robot_config.yaml`;
- `config/state_estimator.yaml`;
- `config/realsense_camera.yaml`;
- `config/realsense_camera_d435i.yaml`;
- `config/heightmap_builder.yaml`.

Частота оценки состояния задаётся `timesteps.yaml: state_estimator_dt`; целевое текущее значение `0.002` с, то есть 500 Гц.

Частота depth/map задаётся `realsense_camera_d435i.yaml: stream.fps` и `stream.publish_fps`; целевое текущее значение 30 Гц.

Параметры карты, rolling-map, фильтрации, градиента, классификации проходимости, extrinsics камеры и `sync.max_sync_dt_sec` берутся из `heightmap_builder.yaml`.

## Архитектура потоков

Модуль реализуется как один executable `state_estimator_hmb` в папке `StateEstimatorHMB/`.

Обязательные потоки:

- `StateEstimatorThread`, 500 Гц: читает последние IMU/servo/contact/pose данные, выполняет sensor fusion, расчёт body state, leg state и GRF/contact, публикует `ROBOT_STATE`, обновляет ring buffer `RobotStateSnapshot`.
- `PoseCameraThread`: читает T265 pose через `librealsense2` и обновляет внутреннюю odometry-структуру без публикации LCM.
- `DepthCameraThread`, до 30 Гц: читает D435i depth через `librealsense2`, строит pointcloud через `rs2::pointcloud`, применяет depth range/downsampling/outlier filtering и передаёт кадр в очередь карты.
- `HeightMapThread`, до 30 Гц: берёт последний depth pointcloud, выбирает ближайший `RobotStateSnapshot` по timestamp, обновляет heightmap/traversability и публикует результат.
- LCM input threads: принимают внешние `IMU_DATA`, `SERVO_STATE`, `CONTACT_SENSOR`.

Все общие данные должны передаваться через mutex, condition variable или атомарные флаги. Чтение частично обновлённых структур запрещено.

## Синхронизация depth и состояния

Каждый `RobotStateSnapshot` и каждый depth-frame имеют `timestamp_ns`.

Для построения карты высот используется состояние робота, ближайшее по времени к последнему depth-frame. Если `abs(depth_ts - state_ts) > heightmap_builder.yaml: sync.max_sync_dt_sec` и `require_recent_robot_state: true`, depth-frame пропускается.

В `robot_state_msg.timestamp` обязательно записывается актуальный Unix time в ns. Поле не должно оставаться нулевым.

Для v1 достаточно выбора ближайшего snapshot. Интерполяция позиции и yaw между двумя snapshot допускается как последующее улучшение.

## Оценка состояния

Алгоритм должен соответствовать текущему `StateEstimator`:

- orientation fusion через `SensorFusion`;
- начальный yaw offset;
- body position с учётом `camera_offset_x/y/z` из `state_estimator.yaml`;
- linear velocity с компенсацией плеча камеры;
- leg state через `LegState`;
- GRF/contact через `GMBasedForceObserver`;
- torque scaling `0.73 / 10.0`, пока этот коэффициент актуален для железа.

Формат `robot_state_msg` должен оставаться совместимым с существующими потребителями.

### Выбор источника позы корпуса

Есть ровно два источника оценки: `kalman` (фильтр `KalmanMIT` по ИМУ и кинематике ног)
и `t265` (одометрия T265). Единственный ключ `robot_state_source` в
`state_estimator.yaml` назначает источник каналу `ROBOT_STATE`; канал
`ROBOT_STATE_CHECK` всегда получает противоположный источник
(`state_estimator_hmb::Other`). Отдельного ключа для check-канала нет: два канала — это
две оценки, которые сравниваются в одном логе, и назначить им один источник бессмысленно.
Ключ необязательный, умолчание — `t265`.

Правила, обязательные для реализации:

- **Ориентация всегда из ИМУ.** `body.orientation`, `body.orientation_quaternion` и
  `body.ang_vel` берутся из ИМУ и от источника не зависят, то есть в обоих каналах
  совпадают. Углы Эйлера и кватернион обязаны описывать один и тот же поворот: из yaw
  вычитается стартовое смещение, поэтому кватернион строится из той же матрицы
  поворота, а не берётся сырым из сообщения ИМУ. Иначе стопы (FK в `LegState` идёт по
  кватерниону) оказались бы в другом yaw-фрейме, чем корпус.
- **Канал формируется одним источником целиком.** Поза, линейная скорость корпуса и всё
  состояние ног (`legs.*_pos/_vel`) канала считаются от одной и той же оценки.
  Смешивания источников внутри одного сообщения быть не должно.
- **Кинематика ног считается один раз за такт** — наблюдатель GRF имеет внутреннее
  состояние, и повторный вызов сдвинул бы его фильтры. Расчёт ведётся при НУЛЕВОЙ базе,
  поэтому его результат ни к одному источнику не привязан: `legs.*_pos` — вектор от
  корпуса до стопы, `legs.*_vel` — скорость стопы относительно корпуса. Мировые величины
  канала получаются прибавлением позы и линейной скорости его источника. Это точное
  равенство, а не приближение: мировая поза стопы равна `p_body + R * p_foot_body`, а
  базовая скорость входит в скорость стопы линейно (тест `leg_state_compose_test`).
- **GRF и контакты от позы не зависят** — они считаются на конфигурации с явно обнулённой
  базой, поэтому их можно передать в `KalmanMIT` до того, как поза будет выбрана. Именно
  это снимает кажущуюся циклическую зависимость «фильтру нужны контакты → контактам нужна
  поза → позу даёт фильтр».
- **Снимок для карты высот** (`PushEstimatorSnapshot`) использует состояние, ушедшее в
  `ROBOT_STATE`, чтобы карта строилась в той же системе координат, что публикуется.
- **Несогласованный конфиг — отказ на старте.** Если `robot_state_source` равен
  `kalman`, а `kalman_mit.enabled: false`, процесс обязан завершиться с ошибкой при
  загрузке конфига, а не молча публиковать T265. На check-канал это не распространяется:
  при выключенном фильтре он просто не публикуется, о чём сказано в стартовом баннере.
- **Источник ещё не валиден — канал не публикуется.** Пока оценка источника невалидна
  (`KalmanMITOutput::valid == false` до первого контакта; нет ни одного кадра T265),
  сообщение в его канал не уходит вовсе. Молчаливой подмены источника не бывает: канал
  либо несёт заявленную оценку, либо молчит. Переход валидности каждого источника
  логируется один раз.
- **Защёлки — по первым данным, а не по первому такту.** Смещение позиции T265 берётся
  на первом кадре одометрии, стартовый yaw — на первом сообщении ИМУ. Защёлка на ещё не
  пришедших (нулевых) данных сместила бы всю траекторию.

## Карта высот и проходимости

Алгоритм переносится из `HeightMapBuilder`:

- преобразование `camera -> body -> world`;
- rolling global heightmap;
- локальное окно вокруг робота;
- сохранение ранее наблюдённых высот, пока область не вытеснена rolling map;
- morphology opening;
- Sobel gradient;
- классификация `STEPPABLE`, `UNSTEPPABLE`, `IMPASSABLE`;
- упаковка ячейки: bit 15 `valid`, bits 14..13 `traversability_class`, bits 12..0 `height_q`.

Depth image в pointcloud должен преобразовываться через `librealsense2`, предпочтительно `rs2::pointcloud`.

Публикуемый `heightmap_msg` сохраняет текущий контракт:

- `origin_x`;
- `origin_y`;
- `yaw`;
- `data_size`;
- `data`.

Публикация `POINCLOUD` управляется `heightmap_builder.yaml: runtime.publish_pointcloud`.

## Обработка ошибок

Модуль должен явно сообщать об отсутствии:

- `CONFIGPATH`;
- `LCM_LOCOMOTION_URL`;
- `LCM_SERVO_URL`.

Если обязательная RealSense-камера не открывается, модуль завершает работу с ошибкой.

500 Гц state loop не должен блокироваться из-за depth camera, построения карты или публикации heightmap.

При `runtime.verbose: true` нужно логировать статистику пропущенных depth-кадров по причине синхронизации.

## Приёмка

Минимальные критерии:

- `StateEstimatorHMB` собирается через CMake;
- модуль стартует с существующими конфигами;
- `ROBOT_STATE` публикуется с частотой 500 Гц;
- `HEIGHTMAP` публикуется с частотой до 30 Гц;
- `robot_state_msg.timestamp` не равен нулю;
- карта использует snapshot состояния с `abs(depth_ts - state_ts) <= sync.max_sync_dt_sec`;
- `runtime.publish_pointcloud: false` отключает публикацию `POINCLOUD`;
- для нового контура не требуется запуск старых `RealsenseCamera`, `RealsenseCameraD435i`, `StateEstimator`, `HeightMapBuilder`.

Рекомендуемые тесты:

- выбор ближайшего `RobotStateSnapshot` по timestamp;
- packing heightmap cell;
- rolling map shift;
- пропуск depth-frame при устаревшем state;
- интеграционный запуск с mock LCM IMU/servo и depth playback;
- smoke test на T265 + D435i с serial из конфигов.

## Дополнительно
- функционал модуля должен быть разделен на логичные классы: обслуживание D435i, обслуживание T265, построение карты высот, оценка контакта и т.д.
