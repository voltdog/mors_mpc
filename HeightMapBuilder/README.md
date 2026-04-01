# HeightMapBuilder

Текущая реализация покрывает шаги 1-6 из `plan.md`:

1. Получение входных данных из LCM:
   - `DEPTH_IMAGE` (`depth_image_msg`)
   - `ROBOT_STATE` (`robot_state_msg`)
2. Предобработка depth:
   - отбрасывание невалидных значений
   - фильтрация по диапазону глубины
   - опциональный downsampling
   - подавление выбросов по локальной медиане
3. Восстановление 3D-точек в системе камеры по intrinsics.
4. Преобразование `camera -> body -> world` по `T_body_camera` и позе робота из `ROBOT_STATE`.
5. Построение внутренней 2.5D карты высот (регулярная XY-сетка в world frame).
6. Публикация локального окна вокруг робота в канал `HEIGHTMAP` (packed `heightmap_msg`).

Результаты публикуются:
- в `POINCLOUD` сообщением `pointcloud_msg` (точки в world frame),
- в `HEIGHTMAP` сообщением `heightmap_msg` (локальное окно 2.5D карты, row-major packed cells).

Параметры положения и наклона камеры берутся из `config/heightmap_builder.yaml`:
- `transforms.camera_frame.position_body_m`
- `transforms.camera_frame.euler_deg`
- `transforms.depth_camera.position_camera_frame_m`
- `transforms.depth_camera.euler_deg`

Параметры 2.5D карты и окна:
- `map.cell_size`
- `map.global_size_x`, `map.global_size_y`
- `map.local_window_cells_x`, `map.local_window_cells_y`
- `map.height_min`, `map.height_max`, `map.height_resolution`
- `channels.heightmap`

## Сборка

```bash
cd HeightMapBuilder
mkdir -p build
cd build
cmake ..
make -j
```

## Запуск

```bash
./height_map_builder
```

По умолчанию конфиг читается из:
- `$CONFIGPATH/heightmap_builder.yaml`, либо
- `../config/heightmap_builder.yaml` (fallback).
