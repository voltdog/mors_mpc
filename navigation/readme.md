# Навигация

Стек [dddmr_navigation](https://github.com/dfl-rlab/dddmr_navigation): lego_loam строит карту,
mcl_3dl локализует на ней, global_planner и p2p_move_base ведут робота к цели. Стек работает в
docker-контейнере `dddmr:jazzy`, симулятор и контроллер запускаются на хосте через `run.sh`.
Пока описана только симуляция.

Карты лежат в `~/dddmr_bags`, в контейнере это `/root/dddmr_bags`.

Каждая команда ниже запускается в отдельном терминале. В терминалах хоста должен быть подключён
`ros_ws/install/setup.bash`.

# Требования
- Ubuntu 24.04
- ROS 2 Jazzy 

## Установка

Запустите скрипт установки:

```bash
navigation/dddmr_install.sh
```

Скрипт ставит docker, клонирует dddmr в `~/dddmr_navigation` на зафиксированный коммит, накладывает
патчи из `navigation/patches`, собирает образ (30-60 минут) и workspace внутри контейнера. Если
скрипт добавил вас в группу docker, перелогиньтесь.

Повторный запуск безопасен. `--rebuild-image` пересобирает образ, `--clean` удаляет build/install/log.
Если в `~/dddmr_navigation` есть правки, которых нет в патчах, скрипт остановится: сохраните их
патчем в `navigation/patches` или откатите.

В `config/simulation.yaml` должно стоять `lidar: True`, иначе `./run.sh --nav` не стартует.

## Картографирование

(Терминал 1) Запустите симуляци в режиме навигации:
```bash
./run.sh --sim --nav
```

(Терминал 2) Запустите контроллер клавиатуры:
```bash
ros2 run mors_keyboard_control mors_keyboard_control
```

(Терминал 3) Пока робот лежит, сохраните стартовую позу. По ней потом задаётся начальная поза на карте.

```bash
scripts/run_dddmr_container.sh bash -c 'ros2 topic echo --once /odom > /root/dddmr_bags/start_odom.yaml'
```

(Терминал 3) Запустите картографирование (откроется rviz):

```bash
scripts/run_dddmr_container.sh ros2 launch mors_dddmr mors_airy_mapping.launch planner:=false
```

С помощью клавиатуры подайте команды встать и идти. Обойдите помещение, чтобы lego_loam построил карту. В rviz можно включить отображение карты и облака точек.

(Терминал 4) Сохраните карту:

```bash
scripts/run_dddmr_container.sh ros2 service call /save_mapped_point_cloud std_srvs/srv/Empty
```

Карта сохранится в `~/dddmr_bags/mors_building_<дата_время>`. Переименуйте её и положите рядом
стартовую позу:

```bash
mv ~/dddmr_bags/mors_building_<дата_время> ~/dddmr_bags/mors_building
mv ~/dddmr_bags/start_odom.yaml ~/dddmr_bags/mors_building/
```

### Фильтрация

Делается один раз для каждой карты. Убирает поверхность пола из `pcd/*_feature.pcd`. Без этого
глобальный планировщик помечает узлы на антресоли и лестнице как lethal и пишет
`Goal was aborted: No path found`.

```bash
.mpc_venv/bin/python navigation/tools/map_check.py clean ~/dddmr_bags/mors_building ~/dddmr_bags/mors_building_clean
```

## Навигация по готовой карте

```bash
./run.sh --sim --nav
scripts/run_dddmr_container.sh ros2 launch mors_dddmr mors_airy_localization.launch map_dir:=/root/dddmr_bags/mors_building_clean
ros2 run mors_keyboard_control mors_keyboard_control
```

Пока робот лежит, задайте начальную позу по ground truth симулятора:

```bash
python3 navigation/tools/init_pose.py --map-dir ~/dddmr_bags/mors_building_clean --std 0.05 2
```

`--std 0.05 2` сужает разброс частиц. С разбросом по умолчанию (как у 3DInitialPose в rviz) курс
в начале может ошибаться на 15-40 градусов, и робот теряет путь.

На клавиатуре перети в режим LOCOMOTION_MODE. Цели задаются в rviz инструментом Publish Point.

## Навигация во время картографирования

```bash
./run.sh --sim --nav
scripts/run_dddmr_container.sh ros2 launch mors_dddmr mors_airy_mapping.launch
ros2 run mors_keyboard_control mors_keyboard_control
```

На клавиатуре перети в режим LOCOMOTION_MODE. Цели задаются в rviz инструментом Publish Point,
и только на пол, который уже есть на карте. Планировщик берёт растущую карту lego_loam и
перестраивает граф при каждом её обновлении. Сохранить карту можно так же, как в разделе
"Картографирование".
