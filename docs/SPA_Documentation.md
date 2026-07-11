# Документация алгоритма SPA (Simplified Pressure Advance) v4.10

## Содержание

1. [Физические основы](#1-физические-основы)
2. [Математическая модель](#2-математическая-модель)
3. [Программная реализация](#3-программная-реализация)
4. [Список файлов](#4-список-файлов)
5. [Сценарии использования](#5-сценарии-использования)
6. [История изменений](#6-история-изменений)

---

## 1. Физические основы

### 1.1 Проблема: упругость филамента

В FDM-печати экструдер представляет собой упругую механическую систему. Филамент (пластиковая нить) сжимается под давлением в хотэнде. Когда головка начинает движение с ускорением, давление в сопле падает — возникает **недоэкструзия** (under-extrusion). При завершении движения давление возрастает — возникает **переэкструзия** (over-extrusion). Этот эффект проявляется в виде дефектов на углах и резких переходах.

```
Скорость печати:          ───────╱╲───────
                                     ╲╱
Давление в сопле:               ╱──────╲
(с задержкой)              ╱──────────────╲

Расход пластика:           ╱──╲    ╱──╲
(с выбросом)              ╱    ╲──╱    ╲
                            недолив  перелив
```

### 1.2 Идея Pressure Advance

**Pressure Advance (PA)** — это метод компенсации, при котором экструдер опережает (или отстаёт от) движение печатающей головки, чтобы поддерживать постоянное давление в сопле.

Классический линейный advance (Marlin LinAdvance) использует модель:
```
E_коррекция = K × V_печати × ускорение
```

Это требует знания ускорения, производной скорости и даёт сильную связь между параметрами движения.

### 1.3 Пропорциональная модель SPA (v4.10+)

**SPA (Simplified Pressure Advance)** реализует **пропорциональную модель** (эквивалент Klipper PA): компенсация вычисляется как прямая пропорция от сырой скорости экструзии:

```
E_компенсация = K × Ve_raw
```

**Ключевые отличия от классического LinAdvance:**
- Не требует знания ускорения блока
- Работает в фиксированном временном кадре `FTM_TS = 1 / FTM_FS`
- Прямая пропорция `offset = K × Ve_raw` (без интегратора) — **нулевая фазовая задержка**
- Математически эквивалентен Klipper Pressure Advance
- **Dynamic Volumetric SRL** предотвращает пропуск шагов, базируясь на максимальном объёмном расходе хотенда
- Плавный сброс offset при ретрактах (`SPA_RETRACT_DECAY`)
- Per-block K (сохраняется при планировании) — устранён race condition при смене K во время движения

---

## 2. Математическая модель

### 2.1 Обозначения

| Символ | Описание | Единицы |
|--------|----------|---------|
| `K` | Коэффициент Pressure Advance | с (секунды) |
| `Ve[t]` | Сырая скорость экструзии в момент t | мм/с |
| `E_planned[t]` | Плановое положение экструдера | мм |
| `E_corrected[t]` | Скорректированное положение экструдера | мм |
| `offset[t]` | Компенсация PA (offset = K × Ve) | мм |
| `offset_max` | Максимальный лимит offset (PA_MAX_P_MM) | мм |
| `Q_max` | Максимальный объёмный расход хотенда (MVS) | мм³/с |
| `V_f_max` | Макс. скорость филамента = Q_max / A_filament | мм/с |
| `A_fil` | Площадь сечения филамента = π × (d/2)² | мм² |
| `FTM_FS` | Частота дискретизации FT Motion | Гц |
| `FTM_TS` | Период кадра = 1 / FTM_FS | с |
| `Q16` | Формат фиксированной точки: 16 дробных битов | — |

### 2.2 Уравнения (v4.10)

**Шаг 1. Вычисление текущей скорости экструзии:**
```
Ve[t] = (E_planned[t] - E_planned[t-1]) × FTM_FS      (1)
```

**Шаг 2. Целевой offset (прямая пропорция, только при экструзии):**
```
target[t] = K × Ve[t]                                    (2)
```

**Шаг 3. Asymmetric Dynamic Volumetric SRL (v4.10):**
```
Δoffset[t] = target[t] - offset[t-1]

avail = V_f_max - |Ve[t]|          # доступный запас скорости филамента
δ_max_кадр = max(0, avail × FTM_TS)  # доступное Δoffset за 1 кадр

if (Δoffset[t] > δ_max_кадр)
    Δoffset[t] = δ_max_кадр
if (Δoffset[t] < -δ_max_кадр)
    Δoffset[t] = -δ_max_кадр

offset[t] = offset[t-1] + Δoffset[t]                     (3)
```

где `V_f_max = Q_max / (π × (d_fil/2)²)` — максимальная скорость филамента.

**Ключевое отличие от обычного SRL (v4.8):** Лимит не константа `SPA_PA_MAX_RATE` (подбираемая эмпирически), а динамическая величина, зависящая от текущего Ve и физического MVS хотенда.

**Шаг 4. Жёсткий clamp по PA_MAX_P_MM:**
```
if (|offset[t]| > offset_max)
    offset[t] = clamp(offset[t], -offset_max, +offset_max)
```

**Шаг 5. Затухание при ретракте (Retract Decay):**
```
Если не pa_extruding:
    offset[t] = offset[t-1] - offset[t-1] >> SPA_RETRACT_DECAY
```

**Шаг 6. Скорректированное положение экструдера:**
```
E_corrected[t] = E_planned[t] + offset[t]                (4)
```

### 2.3 Почему Dynamic Volumetric SRL, а не константа?

| Характеристика | Константный SRL (v4.8) | Dynamic Volumetric SRL (v4.9) |
|---------------|----------------------|------------------------------|
| **Принцип** | Эмпирическая константа `SPA_PA_MAX_RATE` | Физический лимит `Q_max / A_fil` |
| **На разгоне (Ve≈0)** | `δ_max = 0.06 мм/кадр` (300 мм/с × 200 мкс) | **`δ_max = 0.0015 мм/кадр`** (7.48 мм/с × 200 мкс) — зато точно! |
| **На высокой Ve** | `δ_max = 0.06 мм/кадр` | **`δ_max ≈ 0`** (всё Q уже занято базовой подачей) |
| **Гарантия step loss** | Нет (R=300 даёт 28000 steps/s — мотор не успевает) | **Да** (гарантирует, что Ve + d(offset)/dt ≤ Q_max) |
| **Размерность** | мм/с (эмпирическая) | мм³/с (физическая, MVS хотенда) |

**Вывод:** v4.10 решает фундаментальную проблему — гарантирует, что **суммарная скорость филамента (Ve + PA offset rate) никогда не превышает физической способности хотенда проплавлять пластик**, независимо от K и ускорения.

**Асимметричность:** SRL в v4.10 работает асимметрично с коэффициентом SPA_SRL_DECAY_FACTOR
- **Offset растёт (Ve растёт):** полный SRL — `δ_max = avail / FTM_FS`, защита от stall
- **Offset падает (Ve падает):** ослабленный SRL — `δ_max × SPA_SRL_DECAY_FACTOR`, анти-наплыв
- **Торможение (Ve падает):** `avail` увеличивается → offset может падать быстрее (выдавливание излишка)

### 2.4 Анализ размерности

```
offset [мм] = K [с] × Ve [мм/с] = [мм] ✓
```

`K = 0.1` означает, что при скорости экструзии 10 мм/с компенсация составит 1 мм.

### 2.5 Q16 — формат фиксированной точки

Все вычисления SPA выполняются в целочисленной арифметике **Q16** (16 дробных битов).

```
Значение_Q16 = round(значение_float × 2^16) = round(значение_float × 65536)
```

**Используемые переменные в Q16:**

| Переменная | Тип | Описание | Q16 |
|-----------|-----|----------|-----|
| `ftmotion_pa_k_q16` | int32_t | K × 65536 | Q16 |
| `spa_pa_offset_q16` | int64_t | offset[t] × 65536 | Q16 (хранится в int64_t) |
| `pa_max_offset_q16` | int64_t | offset_max × 65536 | Q16 (0 = без лимита) |
| `spa_v_filament_max_q16` | int64_t | V_f_max × 65536 = Q_max / A_fil × 65536 | Q16 |

> **v4.10:** Переменные `spa_max_delta_per_frame_q16` и `ftmotion_pa_set_max_rate()` удалены — заменены на Dynamic Volumetric SRL.

### 2.6 Поведение Asymmetric Dynamic Volumetric SRL (v4.10)

SPA v4.10 использует **прямую пропорцию** `offset = K × Ve_raw` с **Asymmetric Dynamic Volumetric Slew Rate Limiter**, что даёт:

- **Физически обоснованный лимит**: SRL базируется на MVS хотенда, а не на эмпирически подобранной константе
- **Автоматическая адаптация**: при низкой Ve (старт разгона) offset растёт быстро; при высокой Ve — медленно; при Ve ≥ V_f_max — только уменьшается
- **Нулевая фазовая задержка пока avail большой**: на малых Ve Δoffset не клиппируется
- **Чёткие углы**: offset начинает спадать одновременно со снижением Ve — нет задержки сброса давления
- **Устойчивость**: отсутствие интегратора исключает windup; жёсткий clamp по PA_MAX_P_MM работает без побочных эффектов
- **Безопасность**: SRL гарантирует, что общий расход никогда не превысит физических возможностей хотенда
- **Асимметричность**: разгон (рост offset) лимитируется жёстче, торможение (спад offset) — мягче

---

## 3. Программная реализация

### 3.1 Архитектура

SPA интегрирован в систему FT Motion (Fixed-Time Motion). Общая архитектура:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        FTMotion::loop()                                 │
│                                                                         │
│  ┌──────────┐    ┌──────────────────────┐    ┌───────────────────┐     │
│  │planner   │───→│fill_stepper_plan_    │───→│stepping_t (buffer)│     │
│  │(block    │    │buffer()              │    │                   │     │
│  │ queue)   │    │                      │    │ step_pulse gen.   │     │
│  └──────────┘    │  for each frame:     │    └─────────┬─────────┘     │
│                  │    calc_traj_point() │              │                │
│                  │    stepping_enqueue()│              ▼                │
│                  └──────────────────────┘        Stepper ISR           │
│                                                                         │
│         calc_traj_point() содержит SPA (под #if ENABLED(PA_LOOKAHEAD)) │
└─────────────────────────────────────────────────────────────────────────┘
```

### 3.2 Условия компиляции

SPA требует наличия **обоих** макросов в [`Configuration_adv.h`](Marlin/Configuration_adv.h):
- `SIMPLIFIED_PA` — управляет глобальными переменными, функциями установки/сброса
- `PA_LOOKAHEAD` — управляет алгоритмом в `calc_traj_point()` и полями `block_t`

Дополнительные макросы (v4.10):
- ~~`SPA_EMA_ALPHA`~~ — **удалён в v4.7+** (EMA заменён Dynamic Volumetric SRL)
- ~~`SPA_SOFT_CLAMP`~~ — **устарел в v4.6+** (интегратор устранён, windup невозможен)
- ~~`SPA_PA_MAX_RATE`~~ — **удалён в v4.10+** (заменён на `SPA_PA_MAX_VOLFLOW`)
- `SPA_TELEMETRY` — вывод телеметрии
- `SPA_PEAK_TRACKING` — пиковый offset в телеметрии (Task 3)
- `SPA_PA_MAX_VOLFLOW` — Dynamic Volumetric SRL (Task 5, мм³/с, по умолч. 15)
- `SPA_RETRACT_DECAY` — затухание при ретрактах

### 3.3 Глобальные переменные

Определены в [`ft_motion.cpp`](Marlin/src/module/ft_motion.cpp:54):

```cpp
#if ENABLED(SIMPLIFIED_PA)
// === SPA v4.10: Прямая пропорция offset = K × Ve + Asymmetric Dynamic Volumetric SRL ===
int32_t ftmotion_pa_k_q16 = 0;                  // K в Q16 (K_float × 65536)
static int64_t spa_pa_offset_q16 = 0;           // Компенсация PA (Q16 в int64_t), offset = K × Ve
static int64_t pa_max_offset_q16 = int64_t(PA_MAX_P_MM * 65536.0f); // Лимит offset (из конфига)

// Максимальная скорость филамента (V_f_max = Q_max / A_filament) в Q16.
// Определяет предел: |Ve + d(offset)/dt| ≤ V_f_max.
static int64_t spa_v_filament_max_q16 = int64_t(
  (SPA_PA_MAX_VOLFLOW) / ((PI * 0.875f * 0.875f)) * 65536.0f
);

#if ENABLED(SPA_PEAK_TRACKING)
  static int64_t spa_peak_offset_q16 = 0;       // Пиковый |offset| за период телеметрии
#endif
#endif
```

### 3.4 Основные функции

#### Установка коэффициента `K` — [`ftmotion_pa_set_k()`](Marlin/src/module/ft_motion.cpp:65)

```cpp
void ftmotion_pa_set_k(float k_new) {
  ftmotion_pa_k_q16 = int32_t(k_new * 65536.0f);
}
```

Вызывается из `planner.set_advance_k()` (синхронизация SPA с extruder_advance_K[]).

#### Установка лимита offset — [`ftmotion_pa_set_max_offset()`](Marlin/src/module/ft_motion.cpp:71) (Task 4)

```cpp
void ftmotion_pa_set_max_offset(float max_offset_mm) {
  pa_max_offset_q16 = int64_t(max_offset_mm * 65536.0f);
}
```

Управляется через `M900 L<value>`. Диапазон: 0.1–10.0 мм.

#### Установка Dynamic Volumetric SRL — [`ftmotion_pa_set_max_volflow()`](Marlin/src/module/ft_motion.cpp:76) (Task 5)

```cpp
void ftmotion_pa_set_max_volflow(float volflow_mm3_s) {
  spa_v_filament_max_q16 = int64_t(volflow_mm3_s / (PI * 0.875f * 0.875f) * 65536.0f);
}
```

Управляется через `M900 R<value>`. Значение = MVS хотенда [мм³/с].
`R0` = unlimited (только для диагностики).

#### Сброс состояния — [`ftmotion_pa_reset_state()`](Marlin/src/module/ft_motion.cpp:81)

```cpp
void ftmotion_pa_reset_state() {
  spa_pa_offset_q16 = 0;
  #if ENABLED(SPA_PEAK_TRACKING)
    spa_peak_offset_q16 = 0;
  #endif
}
```

#### Основной алгоритм — [`calc_traj_point()`](Marlin/src/module/ft_motion.cpp:532)

Псевдокод секции SIMPLIFIED_PA (внутри `#if ENABLED(PA_LOOKAHEAD)`), **v4.10 — прямая пропорция + Asymmetric Dynamic Volumetric SRL**:

```cpp
block_t* current_block = stepper.get_current_block();

if (current_block && current_block->pa_active) {
  const int32_t block_K_q16 = current_block->pa_K_q16;      // Per-block K
  const float e_planned = traj_coords.e;

  // (1) Ve_raw в Q16
  const int32_t ve_curr_q16 = int32_t((e_planned - prev_traj_e) * 65536.0f * float(FTM_FS));

  if (current_block->pa_extruding) {
    // (2) Целевой offset (прямая пропорция)
    const int64_t target_offset_q16 = ((int64_t)block_K_q16 * (int64_t)ve_curr_q16) >> 16;

    // (3) Δoffset = target - current
    int64_t delta_offset_q16 = target_offset_q16 - spa_pa_offset_q16;

    // (T5) Asymmetric Dynamic Volumetric SRL (SPA v4.10)
    //     offset растёт → полный SRL (защита от stall),
    //     offset падает → ослабленный ×SPA_SRL_DECAY_FACTOR (анти-наплыв).
    if (spa_v_filament_max_q16 > 0) {
      const int64_t ve_abs_q16 = (ve_curr_q16 >= 0) ? ve_curr_q16 : -ve_curr_q16;
      const int64_t avail_q16 = spa_v_filament_max_q16 - ve_abs_q16;
      // δ_max/кадр = V_f_max / FTM_FS (Q16).
      // При avail ≤ 0 (overspeed) используем Vf_max/FS как fallback,
      // чтобы offset мог снижаться (безопасный выход из overspeed).
      const int64_t max_delta_frame_q16 = (avail_q16 > 0)
        ? avail_q16 / int64_t(FTM_FS)
        : spa_v_filament_max_q16 / int64_t(FTM_FS);

      if (delta_offset_q16 > 0) {
        // Offset растёт: полный SRL — защита от пропуска шагов
        if (delta_offset_q16 > max_delta_frame_q16)
          delta_offset_q16 = max_delta_frame_q16;
      } else {
        // Offset падает: ослабленный SRL — микронаплывы на notch exit
        const int64_t decay_limit_q16 = max_delta_frame_q16 * (int64_t)SPA_SRL_DECAY_FACTOR;
        if (delta_offset_q16 < -decay_limit_q16)
          delta_offset_q16 = -decay_limit_q16;
      }
    }

    // (4) Применение ограниченного смещения
    spa_pa_offset_q16 += delta_offset_q16;

    // (5) Жёсткий clamp по PA_MAX_P_MM
    if (pa_max_offset_q16 > 0) {
      if (spa_pa_offset_q16 > pa_max_offset_q16)
        spa_pa_offset_q16 = pa_max_offset_q16;
      else if (spa_pa_offset_q16 < -pa_max_offset_q16)
        spa_pa_offset_q16 = -pa_max_offset_q16;
    }
  }
  else {
    // (6) Retract Decay: затухание при ретракте
    #if SPA_RETRACT_DECAY > 0
      spa_pa_offset_q16 -= spa_pa_offset_q16 >> SPA_RETRACT_DECAY;
    #endif
  }

  // (7) Применение offset
  prev_traj_e = e_planned;
  traj_coords.e += (float)spa_pa_offset_q16 * (1.0f / 65536.0f);
}
```

**Ключевые отличия от v4.9:**
1. **Удалён** константный `spa_max_delta_per_frame_q16` (SPA_PA_MAX_RATE)
2. **Добавлен** Asymmetric Dynamic Volumetric SRL:
   - `δ_max_кадр = max(0, V_f_max - |Ve|) / FTM_FS` (offset растёт, полный SRL)
   - `δ_max_кадр = max(0, V_f_max - |Ve|) / FTM_FS × SPA_SRL_DECAY_FACTOR` (offset падает, ослабленный)
3. **Добавлена** переменная `spa_v_filament_max_q16` — V_f_max на основе `SPA_PA_MAX_VOLFLOW`
4. **Добавлена** функция `ftmotion_pa_set_max_volflow()` — установка через M900 R (мм³/с)
5. **Fallback при overspeed**: при `avail ≤ 0 (Ve ≥ V_f_max)` SRL разрешает падение offset со скоростью `V_f_max / FTM_FS` — безопасный выход из перегруза
6. Физически гарантирует: `Ve + d(offset)/dt ≤ Q_max / A_filament` (на рост),
   и `Ve + d(offset)/dt × 8 ≤ Q_max / A_filament` (на падение, factor=8)
7. **Новая константа**: `SPA_SRL_DECAY_FACTOR` (по умолч. 8) — множитель ветки падения offset

### 3.5 Вспомогательные структуры данных

Определены в [`planner.h`](Marlin/src/module/planner.h:322):

```cpp
#if ENABLED(PA_LOOKAHEAD)
  int32_t pa_K_q16;           // K для этого блока (копия на момент запуска)
  bool pa_active;             // Флаг активности SPA для блока
  bool pa_extruding;          // Флаг: есть шаги экструдера (steps.e > 0)
#endif
```

Поля инициализируются в [`planner.cpp:2490`](Marlin/src/module/planner.cpp:2490):
```cpp
block->pa_K_q16 = int32_t(planner.get_advance_k(extruder) * 65536.0f);
block->pa_active = true;
block->pa_extruding = (block->steps.e > 0);
```

### 3.6 Телеметрия (Task 3)

При включении `SPA_TELEMETRY` и `SPA_PEAK_TRACKING` в терминал выводится:

```
PA|K=0.100 Ve=45.2 Off=1.27 OffPeak=1.50
```

- `OffPeak` — пиковое значение |offset| за последние 50 мс
- Позволяет оператору видеть, не упирается ли offset в `PA_MAX_P_MM`

---

## 4. Список файлов

### 4.1 Конфигурация

| Файл | Строки | Описание |
|------|--------|----------|
| [`Configuration_adv.h`](Marlin/Configuration_adv.h:4904) | Defines: `SIMPLIFIED_PA`, `PA_LOOKAHEAD`, ~~`SPA_EMA_ALPHA`~~ (удалён в v4.7), ~~`SPA_SOFT_CLAMP`~~ (устарел в v4.6), ~~`SPA_PA_MAX_RATE`~~ (удалён в v4.9), `SPA_TELEMETRY`, `SPA_PEAK_TRACKING`, `PA_MAX_P_MM`, `SPA_PA_MAX_VOLFLOW`, `SPA_SRL_DECAY_FACTOR`, `SPA_RETRACT_DECAY` |

### 4.2 Ядро SPA

| Файл | Строки | Описание |
|------|--------|----------|
| [`ft_motion.h`](Marlin/src/module/ft_motion.h:433) | Объявления: `ftmotion_pa_k_q16`, функции установки/чтения |
| [`ft_motion.cpp`](Marlin/src/module/ft_motion.cpp:54) | Глобальные переменные и функции установки/сброса |
| [`ft_motion.cpp`](Marlin/src/module/ft_motion.cpp:532) | Основной алгоритм в `calc_traj_point()` |

### 4.3 Планировщик (block_t)

| Файл | Строки | Описание |
|------|--------|----------|
| [`planner.h`](Marlin/src/module/planner.h:322) | Поля block_t: `pa_K_q16`, `pa_active`, `pa_extruding` |
| [`planner.h`](Marlin/src/module/planner.h:1166) | `pa_flush_queue()` — сброс флагов в очереди |
| [`planner.cpp`](Marlin/src/module/planner.cpp:2489) | Инициализация полей SPA в `buffer_segment()` |

### 4.4 Внешние вызовы

| Файл | Строки | Описание |
|------|--------|----------|
| [`M900.cpp`](Marlin/src/gcode/feature/advance/M900.cpp) | `M900 K<value> L<value> R<value>` — установка параметров |
| [`G92.cpp`](Marlin/src/gcode/geometry/G92.cpp) | `G92 E0` — сброс состояния SPA + флагов очереди |
| [`pause.cpp`](Marlin/src/feature/pause.cpp) | Пауза / M600 — сброс состояния SPA |
| [`stepper.h`](Marlin/src/module/stepper.h) | `get_current_block()` — доступ к текущему блоку |

---

## 5. Сценарии использования

### 5.1 Проверка активации SPA

1. Отправьте `M900` без параметров — ответ должен содержать `Advance K=<значение>`
2. При включённой `SPA_TELEMETRY` в терминале должны отображаться строки `PA|K=...`
3. При изменении K через `M900 K0.15` должен измениться `Advance K` в отчёте

### 5.2 Настройка K

Рекомендуемый метод: печать PA-башни с изменением K от 0.00 до 0.30.

Типичные значения: 0.05–0.30 в зависимости от вязкости пластика, длины хотэнда, скорости печати.

> **Важно:** В G-code команда `M900` чувствительна к регистру. Используйте **латинскую** букву `K`:
> - ✅ `M900 K0.10` — правильно
> - ❌ `M900 К0.10` — неправильно (кириллическая `К` не распознаётся)

### 5.3 Настройка Dynamic Volumetric SRL (Task 5)

**Важнейшее изменение в v4.10:** SRL больше не настраивается произвольной константой. Вместо этого нужно указать **максимальный объёмный расход (MVS)** хотенда для вашего филамента.

`M900 R<max_volflow_mm3_s>` — установка MVS хотенда:

- **Как подобрать значение:**
  1. Измерьте MVS для вашего филамента (калибровка в OrcaSlicer или вручную)
  2. Типичные значения: PETG/PLA 0.4mm nozzle → **10–20 мм³/с**
  3. Установите `M900 R<ваше_MVS>`

- **Примеры:**
  - `M900 R18` — MVS = 18 мм³/с (ваш текущий филамент)
  - `M900 R12` — MVS = 12 мм³/с (консервативно для PETG)
  - `M900 R0` — unlimited (ТОЛЬКО для диагностики, не рекомендуется)

- **Проверка на практике:**
  - Если слышен треск/пропуск шагов — SRL работает корректно, нужно уменьшить K или проверить MVS
  - Если offset явно «не успевает» — ваше MVS может быть занижено

### 5.4 Настройка PA_MAX_P_MM (Task 4)

`M900 L<value>` — динамический лимит offset:
- `M900 L1.0` — жёсткий лимит (для жёстких филаментов)
- `M900 L3.0` — мягкий лимит (для мягких филаментов)
- `M900 L0.0` — без лимита (только для отладки!)

### 5.5 Мониторинг offset

При включённом `SPA_PEAK_TRACKING`:
```
PA|K=0.100 Ve=45.2 Off=1.27 OffPeak=1.50
```

- `Off` — текущий offset
- `OffPeak` — пиковый offset за последние 50 мс
- Если `OffPeak` регулярно > `PA_MAX_P_MM` — необходимо увеличить лимит или уменьшить K

### 5.6 Смена филамента

При `M600` или обрыве филамента вызывается `ftmotion_pa_reset_state()` — сброс накопленного offset и флагов.

### 5.7 Совместимость с другими системами

| Компонент | Совместимость | Примечание |
|-----------|--------------|------------|
| FT Motion | Обязателен | SPA работает только в связке с FT Motion |
| Input Shaping | Полная | SPA применяется до shaping/smoothing |
| Smoothing | Полная | Не влияет на работу сглаживания |
| M900 | Да | K, L, R параметры (E — игнорируется в v4.7+) |
| G92 E0 | Да | Сброс состояния + сброс очереди PA |

---

## 6. История изменений

### v4.10 — текущая версия (рекомендуется)

| Дата | Изменение | Задача |
|------|-----------|--------|
| 2026-07-12 | **Asymmetric SRL:** offset растёт → полный SRL, offset падает → ослабленный ×SPA_SRL_DECAY_FACTOR(8). Исправляет микронаплывы на notch exit | v4.10 |
| 2026-07-12 | **Добавлен** макрос `SPA_SRL_DECAY_FACTOR` (8) — множитель ветки падения offset | Task 5 |
| 2026-07-12 | **Fallback при overspeed:** при `avail ≤ 0` SRL разрешает падение offset со скоростью `V_f_max / FTM_FS` — безопасный выход из перегруза | v4.10 |
| 2026-07-11 | **Кардинальная замена SRL:** константный `SPA_PA_MAX_RATE` → **Dynamic Volumetric SRL** на основе MVS хотенда (`SPA_PA_MAX_VOLFLOW`) | v4.9 |
| 2026-07-11 | Удалён макрос `SPA_PA_MAX_RATE` и переменная `spa_max_delta_per_frame_q16` | Task 5 |
| 2026-07-11 | Добавлен макрос `SPA_PA_MAX_VOLFLOW` и переменная `spa_v_filament_max_q16` (V_f_max = Q_max / A_fil) | Task 5 |
| 2026-07-11 | Добавлена функция `ftmotion_pa_set_max_volflow()` вместо `ftmotion_pa_set_max_rate()` | Task 5 |
| 2026-07-11 | Асимметричный SRL: разгон (рост offset) — жёстче, торможение (спад offset) — мягче | v4.9 |
| 2026-07-11 | SRL теперь гарантирует: `Ve + d(offset)/dt ≤ Q_max / A_filament` — **физически обоснованная защита от step loss** | v4.9 |

### v4.8 — архив

| Дата | Изменение | Задача |
|------|-----------|--------|
| 2026-07-11 | **Добавлен Slew Rate Limiter** — ограничение Δoffset/кадр | v4.8 |
| 2026-07-11 | Добавлен макрос `SPA_PA_MAX_RATE` (удалён в v4.9) | Task 5 |
| 2026-07-11 | Добавлена функция `ftmotion_pa_set_max_rate()` (удалена в v4.9) | Task 5 |
| 2026-07-11 | Алгоритм: `target→Δoffset→limit→accumulate` | v4.8 |

### v4.7 — архив

| Дата | Изменение | Задача |
|------|-----------|--------|
| 2026-07-11 | **Удалён EMA-фильтр** — `offset = K × Ve_raw` (нулевая фазовая задержка) | v4.7 |
| 2026-07-11 | M900 E — помечен как устаревший | v4.7 |

### v4.6 — архив

| Дата | Изменение | Задача |
|------|-----------|--------|
| 2026-07-11 | **Дифференциальная модель → прямая пропорция** `offset[t] = K·Ve_smooth` (эквивалент Klipper PA) | v4.6 |
| 2026-07-11 | Удалён интегратор, `spa_ve_prev_q16`, расчёт dVe | v4.6 |
| 2026-07-11 | `SPA_SOFT_CLAMP` — устарел (windup интегратора невозможен) | v4.6 |

### v4.5-beta2 — архив

| Дата | Изменение | Задача |
|------|-----------|--------|
| 2026-07-11 | Исправлен баг: `spa_ema_alpha_q16` не инициализировался | Аудит |
| 2026-07-11 | Исправлен баг: `pa_max_offset_q16` не инициализировался | Аудит |
| 2026-07-11 | Исправлена гонка: Per-block K вместо глобального | Аудит |
| 2026-07-11 | Удалён мёртвый код в `calc_traj_point()` | Аудит |
| 2026-07-11 | Исправлен дублирующийся блок SPA в M900.cpp | Аудит |
| 2026-07-11 | Явная инициализация K в settings.cpp | Аудит |
| 2026-07-01 | Добавлен EMA-фильтр (`SPA_EMA_ALPHA`) | Task 1 |
| 2026-07-01 | Добавлено мягкое клиппирование (`SPA_SOFT_CLAMP`) | Task 2 |
| 2026-07-01 | Добавлен пиковый offset в телеметрию (`SPA_PEAK_TRACKING`) | Task 3 |
| 2026-07-01 | Добавлен `PA_MAX_P_MM` (`M900 L<value>`) | Task 4 |
| 2026-07-01 | Добавлен Retract Decay (`SPA_RETRACT_DECAY`) | v4.5 |

### v4.4

| Дата | Изменение | Автор |
|------|-----------|-------|
| 2026-07-01 | Раскомментирован `PA_LOOKAHEAD` — исправлен мёртвый код | Аудит |
| 2026-07-01 | `spa_pa_offset_q16`: int32_t → int64_t | Аудит |
| 2026-07-01 | Добавлена `pa_max_offset_q16` | Аудит |

---

*Документация составлена на основе исходного кода Marlin Firmware (ветка bugfix-2.1.x). Алгоритм SPA v4.10 реализует пропорциональную модель offset = K·Ve + Dynamic Volumetric SRL (MVS-базированная защита от step loss), с жёстким clamp по PA_MAX_P_MM и Retract Decay, работающую в фиксированном временном кадре FT Motion. Per-block K устраняет race condition при смене K во время движения. Dynamic Volumetric SRL асимметричен и гарантирует, что суммарный расход никогда не превысит физического MVS хотенда.*
