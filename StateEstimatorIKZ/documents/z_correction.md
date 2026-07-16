## 4. Определение (\Delta z_{\text{map}}) по опорным стопам

### 4.1. Назначение

Переменная

[
\Delta z_{\text{map}}
]

описывает вертикальное рассогласование между:

* положением опорных стоп, вычисленным по кинематике робота;
* высотой поверхности, сохранённой в elevation map.

Если карта и кинематическая модель согласованы, то положение каждой опорной стопы по оси (Z) должно совпадать с высотой карты в той же горизонтальной точке.

---

### 4.2. Входные данные

Для каждой ноги (i) должны быть доступны:

* `contact[i]` — признак подтверждённого контакта с поверхностью;
* ({}^B\mathbf p_{F_i}) — положение стопы относительно корпуса;
* ({}^W R_B) — матрица поворота корпуса в мировой системе координат;
* (z_B^{\text{kin}}) — высота корпуса, полученная по контактной кинематике;
* (x_{F_i}^{\text{map}}, y_{F_i}^{\text{map}}) — горизонтальные координаты стопы в системе карты;
* (H(x,y)) — относительная высота поверхности в elevation map;
* (z_M) — вертикальная координата начала системы elevation map;
* `map_cell_valid[i]` — признак валидности ячейки карты под стопой.

---

### 4.3. Кинематическая высота стопы

Для каждой опорной ноги вычислить вертикальную координату стопы по кинематике:

[
z_{F_i}^{\text{kin}}
====================

z_B^{\text{kin}}
+
\left(
{}^W R_B,{}^B\mathbf p_{F_i}
\right)_z.
]

Здесь

[
\left(
{}^W R_B,{}^B\mathbf p_{F_i}
\right)_z
]

является вертикальной составляющей вектора от корпуса до стопы в мировой системе координат.

---

### 4.4. Высота поверхности по карте

Высота поверхности под стопой в мировой системе координат вычисляется как

[
z_{F_i}^{\text{map}}
====================

z_M
+
H\left(
x_{F_i}^{\text{map}},
y_{F_i}^{\text{map}}
\right).
]

Если elevation map хранит абсолютные высоты, а не высоты относительно (z_M), необходимо использовать

[
z_{F_i}^{\text{map}}
====================

H\left(
x_{F_i}^{\text{map}},
y_{F_i}^{\text{map}}
\right).
]

ИИ-агент должен использовать только один из этих вариантов в зависимости от формата хранения карты.

---

### 4.5. Остаточная ошибка для одной ноги

Для каждой надёжной опорной ноги вычислить residual:

[
r_i
===

## z_{F_i}^{\text{kin}}

z_{F_i}^{\text{map}}.
]

Интерпретация:

* (r_i \approx 0) — карта и кинематика согласованы;
* (r_i > 0) — поверхность карты расположена ниже кинематического положения стопы;
* (r_i < 0) — поверхность карты расположена выше кинематического положения стопы.

---

### 4.6. Выбор допустимых ног

Residual разрешается использовать только при выполнении всех условий:

```text
contact[i] == true
contact_anchor[i].valid == true
map_cell_valid[i] == true
foot_is_slipping[i] == false
foot_is_near_touchdown[i] == false
foot_is_near_liftoff[i] == false
```

Дополнительно рекомендуется проверить скорость стопы:

[
\left|
{}^W\mathbf v_{F_i}
\right|
<
v_{\text{foot,max}}.
]

Если условие не выполнено, residual данной ноги не должен участвовать в расчёте (\Delta z_{\text{map}}).

---

### 4.7. Расчёт общего вертикального рассогласования

Сформировать множество residual допустимых опорных ног:

[
\mathcal R
==========

\left{
r_i
\mid
i \in \mathcal C_{\text{valid}}
\right}.
]

Если допустимых ног меньше двух:

```text
if valid_residual_count < 2:
    delta_z_map_is_valid = false
    do not correct map_origin_z
```

Если допустимых ног не менее двух, вычислить:

[
\Delta z_{\text{map}}
=====================

\operatorname{median}(\mathcal R).
]

Медиана используется вместо среднего значения, поскольку она менее чувствительна к:

* ошибочной ячейке карты;
* постановке стопы на границу ступени;
* локальному выбросу глубины;
* неверно определённому контакту одной ноги.

---

### 4.8. Проверка согласованности residual

Перед применением коррекции необходимо проверить, что residual разных ног описывают общий вертикальный сдвиг карты.

Вычислить разброс:

[
s_r
===

## \max(\mathcal R)

\min(\mathcal R).
]

Коррекция считается допустимой, если

[
s_r
<
s_{\text{max}}.
]

Начальное рекомендуемое значение:

[
s_{\text{max}}
==============

0.02\text{–}0.03\ \text{м}.
]

Логика:

```text
if residual_spread < max_residual_spread:
    delta_z_map_is_valid = true
else:
    delta_z_map_is_valid = false
```

Большой разброс residual означает, что ошибка, вероятно, связана не с общим вертикальным сдвигом карты, а с локальной ошибкой поверхности, проскальзыванием или неверной оценкой контакта.

---

### 4.9. Псевдокод расчёта

```cpp
std::vector<double> residuals;

for (int i = 0; i < NUM_LEGS; ++i)
{
    if (!contact[i])
        continue;

    if (!contact_anchor[i].valid)
        continue;

    if (!map.isValid(
            foot_position_map[i].x(),
            foot_position_map[i].y()))
        continue;

    if (foot_is_slipping[i])
        continue;

    if (foot_is_near_touchdown[i] ||
        foot_is_near_liftoff[i])
        continue;

    const Eigen::Vector3d foot_offset_world =
        R_world_base * foot_position_base[i];

    const double foot_z_kin =
        base_z_kin + foot_offset_world.z();

    const double terrain_height_relative =
        map.getElevation(
            foot_position_map[i].x(),
            foot_position_map[i].y());

    const double foot_z_map =
        map_origin_z + terrain_height_relative;

    const double residual =
        foot_z_kin - foot_z_map;

    residuals.push_back(residual);
}

bool delta_z_map_is_valid = false;
double delta_z_map = 0.0;

if (residuals.size() >= 2)
{
    const double residual_min =
        *std::min_element(
            residuals.begin(),
            residuals.end());

    const double residual_max =
        *std::max_element(
            residuals.begin(),
            residuals.end());

    const double residual_spread =
        residual_max - residual_min;

    if (residual_spread < max_residual_spread)
    {
        delta_z_map = median(residuals);
        delta_z_map_is_valid = true;
    }
}
```

---

## 5. Применение знака коррекции

### 5.1. Определение знака

Вертикальное рассогласование определяется как

[
\boxed{
\Delta z_{\text{map}}
=====================

## z_F^{\text{kin}}

z_F^{\text{map}}
}
]

Знак коррекции следует трактовать строго следующим образом.

#### Положительное значение

Если

[
\Delta z_{\text{map}}>0,
]

то кинематическое положение стопы находится выше поверхности карты.

Следовательно, карта расположена слишком низко, и её вертикальный origin необходимо увеличить.

#### Отрицательное значение

Если

[
\Delta z_{\text{map}}<0,
]

то кинематическое положение стопы находится ниже поверхности карты.

Следовательно, карта расположена слишком высоко, и её вертикальный origin необходимо уменьшить.

---

### 5.2. Правило обновления

Если высоты ячеек хранятся относительно вертикального origin карты, необходимо выполнить:

[
\boxed{
z_M
\leftarrow
z_M
+
\Delta z_{\text{map}}
}
]

В программной реализации:

```cpp
map_origin_z += delta_z_map;
```

Необходимо использовать знак `+`, поскольку сама величина (\Delta z_{\text{map}}) уже определена как разность

```text
kinematic foot height - map foot height
```

---

### 5.3. Проверка знака на численном примере

Пусть кинематическая высота стопы равна

[
z_F^{\text{kin}}
================

0.10\ \text{м},
]

а карта показывает

[
z_F^{\text{map}}
================

0.03\ \text{м}.
]

Тогда

[
\Delta z_{\text{map}}
=====================

# 0.10-0.03

0.07\ \text{м}.
]

Текущее начало карты:

[
z_M=0.
]

После коррекции:

[
z_M^{\text{new}}
================

# 0+0.07

0.07\ \text{м}.
]

Новая высота поверхности карты:

[
z_F^{\text{map,new}}
====================

z_M^{\text{new}}
+
H(x_F,y_F).
]

Если

[
H(x_F,y_F)=0.03\ \text{м},
]

то

[
z_F^{\text{map,new}}
====================

# 0.07+0.03

0.10\ \text{м}.
]

После коррекции карта совпадает с кинематическим положением стопы.

---

### 5.4. Пример отрицательной коррекции

Пусть

[
z_F^{\text{kin}}
================

0.02\ \text{м},
]

а карта показывает

[
z_F^{\text{map}}
================

0.07\ \text{м}.
]

Тогда

[
\Delta z_{\text{map}}
=====================

# 0.02-0.07

-0.05\ \text{м}.
]

Обновление:

[
z_M^{\text{new}}
================

z_M-0.05.
]

Таким образом, вся карта опускается на 5 см.

---

### 5.5. Ограничение величины коррекции

На реальном роботе не рекомендуется применять полную коррекцию за один такт управления.

Следует ограничить изменение:

[
\Delta z_{\text{applied}}
=========================

\operatorname{clamp}
\left(
\Delta z_{\text{map}},
-\Delta z_{\max},
\Delta z_{\max}
\right).
]

После этого:

[
z_M
\leftarrow
z_M
+
\Delta z_{\text{applied}}.
]

Псевдокод:

```cpp
if (delta_z_map_is_valid)
{
    const double delta_z_applied =
        std::clamp(
            delta_z_map,
            -max_map_z_correction_per_cycle,
            max_map_z_correction_per_cycle);

    map_origin_z += delta_z_applied;
}
```

---

### 5.6. Запрет на изменение значений ячеек

Если elevation map хранит относительные высоты

[
H[i,j]
======

z_{\text{terrain}}-z_M,
]

то при изменении `map_origin_z` значения `H[i][j]` изменять не нужно.

Правильная операция:

```cpp
map_origin_z += delta_z_applied;
```

Неправильная операция:

```cpp
for each cell:
    cell.elevation += delta_z_applied;
```

Одновременное изменение `map_origin_z` и высот всех ячеек приведёт к двойной коррекции.

---

### 5.7. Итоговая логика для ИИ-агента

```text
1. Для каждой надёжной опорной ноги вычислить:
       foot_z_kin
       foot_z_map

2. Вычислить:
       residual[i] = foot_z_kin - foot_z_map

3. Проверить:
       количество residual >= 2
       residual имеют малый разброс

4. Вычислить:
       delta_z_map = median(residuals)

5. Ограничить коррекцию:
       delta_z_applied =
           clamp(delta_z_map,
                 -max_correction,
                 +max_correction)

6. Обновить:
       map_origin_z += delta_z_applied

7. Не изменять elevation всех ячеек,
   если они хранятся относительно map_origin_z.
```
