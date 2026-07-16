# Техническое задание на модуль `StateEstimator1DKF`

## Назначение

`StateEstimator1DKF` объединяет в одном C++20-процессе функции модулей `RealsenseCamera`, `RealsenseCameraD435i`, `StateEstimator` и `HeightMapBuilder`.

Цель объединения - убрать внутренние задержки и рассинхрон, возникающие при передаче промежуточных `ODOMETRY`, `DEPTH_IMAGE` и `ROBOT_STATE` через LCM между отдельными процессами. LCM должен использоваться только на внешней границе модуля: для входных датчиков робота и публикации результатов.

## Входы и выходы

Входные LCM-каналы берутся из `config/channels.yaml`:

- `IMU_DATA`;
- `SERVO_STATE`;
- `GAIT_PHASE`.

Внутренние источники RealSense:

- T265 pose stream, serial из `config/realsense_camera.yaml`;
- D435i depth stream, параметры из `config/realsense_camera_d435i.yaml`.

Выходные LCM-каналы:

- `ROBOT_STATE` с типом `mors_msgs::robot_state_msg`;
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

Модуль реализуется как один executable `state_estimator_1d_kf` в папке `StateEstimator1DKF/`.

Обязательные потоки:

- `StateEstimatorThread`, 500 Гц: читает последние IMU/servo/gait phase/pose данные, выполняет sensor fusion, расчёт body state, leg state, GRF и вертикальный contact-aided KF, публикует `ROBOT_STATE`, обновляет ring buffer `RobotStateSnapshot`.
- `PoseCameraThread`: читает T265 pose через `librealsense2` и обновляет внутреннюю odometry-структуру без публикации LCM.
- `DepthCameraThread`, до 30 Гц: читает D435i depth через `librealsense2`, строит pointcloud через `rs2::pointcloud`, применяет depth range/downsampling/outlier filtering и передаёт кадр в очередь карты.
- `HeightMapThread`, до 30 Гц: берёт последний depth pointcloud, выбирает ближайший `RobotStateSnapshot` по timestamp, обновляет heightmap/traversability и публикует результат.
- LCM input threads: принимают внешние `IMU_DATA`, `SERVO_STATE`, `GAIT_PHASE`.

Все общие данные должны передаваться через mutex, condition variable или атомарные флаги. Чтение частично обновлённых структур запрещено.

## Синхронизация depth и состояния

Каждый `RobotStateSnapshot` и каждый depth-frame имеют `timestamp_ns`.

Для построения карты высот используется состояние робота, ближайшее по времени к последнему depth-frame. Если `abs(depth_ts - state_ts) > heightmap_builder.yaml: sync.max_sync_dt_sec` и `require_recent_robot_state: true`, depth-frame пропускается.

В `robot_state_msg.timestamp` обязательно записывается актуальный Unix time в ns. Поле не должно оставаться нулевым.

Для v1 достаточно выбора ближайшего snapshot. Интерполяция позиции и yaw между двумя snapshot допускается как последующее улучшение.

## Оценка состояния

Ориентация, X/Y, работа с камерами и расчёт состояния ног соответствуют
`StateEstimatorMK`:

- orientation fusion через `SensorFusion`;
- начальный yaw offset;
- body position с учётом `camera_offset_x/y/z` из `state_estimator.yaml`;
- linear velocity с компенсацией плеча камеры;
- leg state через `LegState`;
- GRF/contact через `GMBasedForceObserver`;
- leg odometry использует ногу как опорную при `GAIT_PHASE.phase == STANCE || GAIT_PHASE.phase == EARLY_CONTACT`;
- torque scaling `0.73 / 10.0`, пока этот коэффициент актуален для железа.

Вертикальный фильтр имеет состояние
`[z, vz, acceleration_bias_z, foot_height_R1, foot_height_L1,
foot_height_R2, foot_height_L2]`.

- ускорение IMU является входом модели, а не измерением отдельного состояния;
- bias ускорения моделируется случайным блужданием;
- T265 задаёт начальный вертикальный origin;
- повторный Kalman update выполняется только для нового pose frame T265;
- `T265 Z` по умолчанию не используется после инициализации, `T265 VZ`
  остаётся вспомогательным измерением;
- leg velocity и constraint неподвижной стопы добавляются только при валидной
  подтверждённой опоре;
- при отсутствии опор leg velocity не заменяется данными T265;
- новый foot anchor добавляется с переносом cross-covariance из `z` и не
  используется как независимое измерение на том же сэмпле;
- включение и выключение опоры имеют debounce;
- силы контакта восстанавливаются через damped SVD solve для `J^T`, без
  явного обращения Якобиана;
- все параметры с суффиксом `_std` являются стандартными отклонениями.

В `ROBOT_STATE` публикуются X/Y от T265 и Z/VZ от вертикального фильтра.
`contact_states` содержит физический GRF-контакт из `LegState`, без маскирования
фазой походки. Debounced support mask используется только внутри вертикального KF.
`ROBOT_STATE_CHECK` содержит нефильтрованное состояние T265 для сравнения.

Формат `robot_state_msg` должен оставаться совместимым с существующими потребителями.

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

Карта строится из отфильтрованного состояния, но не подаётся обратно в
вертикальный KF как независимое измерение: такая обратная связь была бы
коррелирована с текущим `ROBOT_STATE.Z`.

## Обработка ошибок

Модуль должен явно сообщать об отсутствии:

- `CONFIGPATH`;
- `LCM_CONTROL_URL`;
- `LCM_SERVO_URL`;
- `LCM_VISION_URL`.

Если обязательная RealSense-камера не открывается, модуль завершает работу с ошибкой.

500 Гц state loop не должен блокироваться из-за depth camera, построения карты или публикации heightmap.

При `runtime.verbose: true` нужно логировать статистику пропущенных depth-кадров по причине синхронизации.

## Приёмка

Минимальные критерии:

- `StateEstimator1DKF` собирается через CMake;
- модуль стартует с существующими конфигами;
- `ROBOT_STATE` публикуется с частотой 500 Гц;
- `HEIGHTMAP` публикуется с частотой до 30 Гц;
- `robot_state_msg.timestamp` не равен нулю;
- карта использует snapshot состояния с `abs(depth_ts - state_ts) <= sync.max_sync_dt_sec`;
- `runtime.publish_pointcloud: false` отключает публикацию `POINCLOUD`;
- `vertical_kalman_filter_test` проверяет удержание статической высоты,
  оценивание bias ускорения и innovation gate;
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
