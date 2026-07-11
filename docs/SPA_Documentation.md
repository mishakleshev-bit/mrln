# Документация алгоритма SPA (Simplified Pressure Advance) v4.6

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

### 1.3 Пропорциональная модель SPA (v4.6+)

**SPA (Simplified Pressure Advance)** реализует **пропорциональную модель** (эквивалент Klipper PA): компенсация вычисляется как прямая пропорция от сглаженной скорости экструзии:

```
E_компенсация = K × Ve_smooth
```

**Ключевые отличия от классического LinAdvance:**
- Использует EMA-фильтр низких частот для Ve (`SPA_EMA_ALPHA`) — Task 1
- Не требует знания ускорения блока
- Работает в фиксированном временном кадре `FTM_TS = 1 / FTM_FS`
- Прямая пропорция `offset = K × Ve_smooth` (без интегратора) — **нулевая фазовая задержка**
- Математически эквивалентен Klipper Pressure Advance
- Плавный сброс offset при ретрактах (`SPA_RETRACT_DECAY`)
- Per-block K (сохраняется при планировании) — устранён race condition при смене K во время движения

---

## 2. Математическая модель

### 2.1 Обозначения

| Символ | Описание | Единицы |
|--------|----------|---------|
| `K` | Коэффициент Pressure Advance | с (секунды) |
| `Ve[t]` | Сырая скорость экструзии в момент t | мм/с |
| `Ve_smooth[t]` | Сглаженная скорость (после EMA) | мм/с |
| `α` | Коэффициент EMA-фильтра (SPA_EMA_ALPHA) | — |
| `E_planned[t]` | Плановое положение экструдера | мм |
| `E_corrected[t]` | Скорректированное положение экструдера | мм |
| `offset[t]` | Компенсация PA (offset = K × Ve_smooth) | мм |
| `offset_max` | Максимальный лимит offset (PA_MAX_P_MM) | мм |
| `FTM_FS` | Частота дискретизации FT Motion | Гц |
| `FTM_TS` | Период кадра = 1 / FTM_FS | с |
| `Q16` | Формат фиксированной точки: 16 дробных битов | — |

### 2.2 Уравнения (v4.6)

**Шаг 1. Вычисление текущей скорости экструзии:**
```
Ve[t] = (E_planned[t] - E_planned[t-1]) × FTM_FS      (1)
```

**Шаг 1a. EMA-фильтр низких частот (Task 1):**
```
Ve_smooth[t] = Ve_smooth[t-1] + α × (Ve[t] - Ve_smooth[t-1])
```
где `α = SPA_EMA_ALPHA`. Постоянная времени фильтра:
```
τ ≈ (1 - α) / (α × FTM_FS)
```
При `α = 0.15` и `FTM_FS = 5000 Гц`: `τ ≈ 1.13 мс`.

**Шаг 2. Прямая пропорция offset = K × Ve_smooth (только при экструзии):**
```
offset[t] = K × Ve_smooth[t]                              (2)
```

**Шаг 2a. Жёсткий clamp по PA_MAX_P_MM:**
```
if (|offset[t]| > offset_max)
    offset[t] = clamp(offset[t], -offset_max, +offset_max)
```

**Шаг 2b. Затухание при ретракте (Retract Decay):**
```
Если не pa_extruding:
    offset[t] = offset[t-1] - offset[t-1] >> SPA_RETRACT_DECAY
```

**Шаг 3. Скорректированное положение экструдера:**
```
E_corrected[t] = E_planned[t] + offset[t]                (3)
```

**Ключевое отличие от v4.5:** Вместо дифференциальной модели (ΔVe → интегратор → offset) используется прямая пропорция `offset = K × Ve_smooth`. Это устраняет фазовую задержку ~1.13 мс, которая была причиной недоэкструзии на разгонах и переэкструзии в углах. Мягкое клиппирование (`SPA_SOFT_CLAMP`) больше не требуется — интегратор устранён, windup невозможен.

### 2.3 Зачем нужно EMA-сглаживание?

**Проблема:** В Klipper и современных слайсерах траектория разбита на микро-сегменты. Если применять `K × Ve` к "сырой" скорости каждого микро-сегмента, скорость экструзии будет иметь пилообразный характер.

**Последствия без фильтра:**
1. Экструдерный мотор получает рывки (jerk) на каждом шаге планировщика
2. Возникает **резонанс механики экструдера**
3. **Потеря шагов (step loss)** на высоких скоростях
4. **"Мыльные" углы** из-за микро-флуктуаций давления

**Решение:** EMA-фильтр низких частот сглаживает Ve перед вычислением ΔVe. Это эквивалентно подходу Klipper, где profile velocity smoothing применяется до PA.

### 2.4 Анализ размерности

```
offset [мм] = K [с] × Ve_smooth [мм/с] = [мм] ✓
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
| `spa_ve_smooth_q16` | int32_t | Ve_smooth[t] × 65536 | Q16 |
| `spa_ema_alpha_q16` | int32_t | α × 65536 | Q16 |
| `spa_pa_offset_q16` | int64_t | offset[t] × 65536 | Q16 (хранится в int64_t) |
| `pa_max_offset_q16` | int64_t | offset_max × 65536 | Q16 (0 = без лимита) |

### 2.6 Поведение прямой пропорции (v4.6)

SPA v4.6 использует **прямую пропорцию** `offset = K × Ve_smooth`, что даёт:
- **Нулевая фазовая задержка**: offset вычисляется мгновенно из текущей сглаженной скорости, без ожидания накопления интегратора
- **Автоматический возврат к нулю**: при остановке движения (Ve_smooth = 0) offset = 0
- **Корректная компенсация на разгонах**: offset мгновенно следует за Ve_smooth — нет недоэкструзии в начале движения
- **Чёткие углы**: offset начинает спадать одновременно со снижением Ve_smooth — нет задержки сброса давления
- **Устойчивость**: отсутствие интегратора исключает windup; жёсткий clamp по PA_MAX_P_MM работает без побочных эффектов

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

Дополнительные макросы (v4.6):
- `SPA_EMA_ALPHA` — коэффициент EMA-фильтра (Task 1)
- ~~`SPA_SOFT_CLAMP`~~ — **устарел в v4.6+** (интегратор устранён, windup невозможен; используется жёсткий clamp)
- `SPA_TELEMETRY` — вывод телеметрии
- `SPA_PEAK_TRACKING` — пиковый offset в телеметрии (Task 3)
- `SPA_RETRACT_DECAY` — затухание при ретрактах

### 3.3 Глобальные переменные

Определены в [`ft_motion.cpp`](Marlin/src/module/ft_motion.cpp:54):

```cpp
#if ENABLED(SIMPLIFIED_PA)
// === SPA v4.6: Прямая пропорция offset = K × Ve_smooth (эквивалентно Klipper PA) ===
int32_t ftmotion_pa_k_q16 = 0;                  // K в Q16 (K_float × 65536)
static int32_t spa_ve_smooth_q16 = 0;           // Сглаженная скорость (Q16) — EMA-фильтр
static int64_t spa_pa_offset_q16 = 0;           // Компенсация PA (Q16 в int64_t), offset = K × Ve_smooth
static int64_t pa_max_offset_q16 = int64_t(PA_MAX_P_MM * 65536.0f);  // Лимит offset (из конфига)
int32_t spa_ema_alpha_q16 = int32_t(SPA_EMA_ALPHA * 65536.0f);       // Коэффициент EMA (Q16) из конфига

#if ENABLED(SPA_PEAK_TRACKING)
  static int64_t spa_peak_offset_q16 = 0;       // Пиковый |offset| за период телеметрии
#endif
#endif
```

> **Важно:** `spa_ema_alpha_q16` и `pa_max_offset_q16` инициализируются из конфига при старте и НЕ сбрасываются при смене K через M900.

### 3.4 Основные функции

#### Установка коэффициента `K` — [`ftmotion_pa_set_k()`](Marlin/src/module/ft_motion.cpp:68)

```cpp
void ftmotion_pa_set_k(float k_new) {
  ftmotion_pa_k_q16 = int32_t(k_new * 65536.0f);
}
```

Вызывается из `planner.set_advance_k()` (синхронизация SPA с extruder_advance_K[]).

#### Установка лимита offset — [`ftmotion_pa_set_max_offset()`](Marlin/src/module/ft_motion.cpp:80) (Task 4)

```cpp
void ftmotion_pa_set_max_offset(float max_offset_mm) {
  pa_max_offset_q16 = int64_t(max_offset_mm * 65536.0f);
}
```

Управляется через `M900 L<value>`. Диапазон: 0.1–10.0 мм.

#### Установка EMA-альфа — [`ftmotion_pa_set_ema_alpha()`](Marlin/src/module/ft_motion.cpp:85) (Task 1)

```cpp
void ftmotion_pa_set_ema_alpha(float alpha) {
  LIMIT(alpha, 0.0f, 1.0f);
  spa_ema_alpha_q16 = int32_t(alpha * 65536.0f);
}
```

Управляется через `M900 E<alpha>`. Диапазон: 0.0–1.0.

#### Сброс состояния — [`ftmotion_pa_reset_state()`](Marlin/src/module/ft_motion.cpp:91)

```cpp
void ftmotion_pa_reset_state() {
  spa_ve_smooth_q16 = 0;
  spa_pa_offset_q16 = 0;
  #if ENABLED(SPA_PEAK_TRACKING)
    spa_peak_offset_q16 = 0;
  #endif
}
```

#### Основной алгоритм — [`calc_traj_point()`](Marlin/src/module/ft_motion.cpp:547)

Псевдокод секции SIMPLIFIED_PA (внутри `#if ENABLED(PA_LOOKAHEAD)`), **v4.6 — прямая пропорция**:

```cpp
block_t* current_block = stepper.get_current_block();

if (current_block && current_block->pa_active) {
  const int32_t block_K_q16 = current_block->pa_K_q16;      // Per-block K
  const float e_planned = traj_coords.e;

  // (1) Ve_raw в Q16
  const int32_t ve_curr_raw_q16 = int32_t((e_planned - prev_traj_e) * 65536.0f * float(FTM_FS));

  // (T1) EMA-фильтр: ve_smooth += alpha * (ve_raw - ve_smooth)
  const int32_t ve_diff_q16 = ve_curr_raw_q16 - spa_ve_smooth_q16;
  spa_ve_smooth_q16 += int32_t((int64_t(spa_ema_alpha_q16) * int64_t(ve_diff_q16)) >> 16);

  if (current_block->pa_extruding) {
    // (2) Прямая пропорция: offset = K × Ve_smooth
    const int64_t pa_offset_new_q16 = ((int64_t)block_K_q16 * (int64_t)spa_ve_smooth_q16) >> 16;

    // (3) Жёсткий clamp по PA_MAX_P_MM (мягкое клиппирование не требуется — нет интегратора)
    if (pa_max_offset_q16 > 0) {
      if (pa_offset_new_q16 > pa_max_offset_q16)
        spa_pa_offset_q16 = pa_max_offset_q16;
      else if (pa_offset_new_q16 < -pa_max_offset_q16)
        spa_pa_offset_q16 = -pa_max_offset_q16;
      else
        spa_pa_offset_q16 = pa_offset_new_q16;
    } else {
      spa_pa_offset_q16 = pa_offset_new_q16;
    }
  }
  else {
    // (4) Retract Decay: затухание при ретракте (по-прежнему накопленный offset)
    #if SPA_RETRACT_DECAY > 0
      spa_pa_offset_q16 -= spa_pa_offset_q16 >> SPA_RETRACT_DECAY;
    #endif
  }

  // (5) Применение offset
  prev_traj_e = e_planned;
  traj_coords.e += (float)spa_pa_offset_q16 * (1.0f / 65536.0f);
}
```

**Ключевые отличия от v4.5:**
1. **Удалён** расчёт `dve_q16 = spa_ve_smooth_q16 - spa_ve_prev_q16` — больше не нужен
2. **Удалена** переменная `spa_ve_prev_q16` — хранение предыдущего Ve_smooth не требуется
3. **Удалён** интегратор `spa_pa_offset_q16 += pa_delta_q16` — offset НЕ накапливается, а перезаписывается
4. **Удалено** мягкое клиппирование (`SPA_SOFT_CLAMP`) — с прямым расчётом offset windup невозможен
5. **offset вычисляется мгновенно** как `K × Ve_smooth` — нулевая фазовая задержка
6. `pa_offset_new_q16` — **локальная** переменная; при экструзии offset полностью пересчитывается каждый кадр

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
PA|K=0.100 Ve=45.2 dVe=12.7 Off=1.27 OffPeak=1.50 alpha=0.15
```

- `OffPeak` — пиковое значение |offset| за последние 50 мс
- Позволяет оператору видеть, не упирается ли offset в `PA_MAX_P_MM`

---

## 4. Список файлов

### 4.1 Конфигурация

| Файл | Строки | Описание |
|------|--------|----------|
| [`Configuration_adv.h`](Marlin/Configuration_adv.h:4904) | Defines: `SIMPLIFIED_PA`, `PA_LOOKAHEAD`, `SPA_EMA_ALPHA`, ~~`SPA_SOFT_CLAMP`~~ (устарел в v4.6+), `SPA_TELEMETRY`, `SPA_PEAK_TRACKING`, `PA_MAX_P_MM`, `SPA_RETRACT_DECAY` |

### 4.2 Ядро SPA

| Файл | Строки | Описание |
|------|--------|----------|
| [`ft_motion.h`](Marlin/src/module/ft_motion.h:433) | Объявления: `ftmotion_pa_k_q16`, `spa_ema_alpha_q16`, функции установки/чтения |
| [`ft_motion.cpp`](Marlin/src/module/ft_motion.cpp:54) | Глобальные переменные и функции установки/сброса |
| [`ft_motion.cpp`](Marlin/src/module/ft_motion.cpp:547) | Основной алгоритм в `calc_traj_point()` |

### 4.3 Планировщик (block_t)

| Файл | Строки | Описание |
|------|--------|----------|
| [`planner.h`](Marlin/src/module/planner.h:322) | Поля block_t: `pa_K_q16`, `pa_active`, `pa_extruding` |
| [`planner.h`](Marlin/src/module/planner.h:1166) | `pa_flush_queue()` — сброс флагов в очереди |
| [`planner.cpp`](Marlin/src/module/planner.cpp:2489) | Инициализация полей SPA в `buffer_segment()` |

### 4.4 Внешние вызовы

| Файл | Строки | Описание |
|------|--------|----------|
| [`M900.cpp`](Marlin/src/gcode/feature/advance/M900.cpp) | `M900 K<value> L<value> E<alpha>` — установка параметров |
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

### 5.3 Настройка EMA-фильтра (Task 1)

`M900 E<alpha>` — меняет коэффициент EMA на лету:
- `M900 E0.0` — без фильтрации (поведение v4.4)
- `M900 E0.15` — умеренная фильтрация (рекомендуется для PLA)
- `M900 E0.30` — сильная фильтрация (для PETG/TPU с высокими ускорениями)

**Правило:** Если `OffPeak` в телеметрии регулярно достигает `PA_MAX_P_MM`, увеличьте `alpha` (больше сглаживания) или уменьшите ускорения.

### 5.4 Настройка PA_MAX_P_MM (Task 4)

`M900 L<value>` — динамический лимит offset:
- `M900 L1.0` — жёсткий лимит (для жёстких филаментов)
- `M900 L3.0` — мягкий лимит (для мягких филаментов)
- `M900 L0.0` — без лимита (только для отладки!)

### 5.5 Мониторинг offset

При включённом `SPA_PEAK_TRACKING`:
```
PA|K=0.100 Ve=45.2 dVe=12.7 Off=1.27 OffPeak=1.50 alpha=0.15
```

- `Off` — текущий offset
- `OffPeak` — пиковый offset за последние 50 мс
- Если `OffPeak` регулярно > `PA_MAX_P_MM` — необходимо увеличить лимит или уменьшить K

### 5.6 Смена филамента

При `M600` или обрыве филамента вызывается `ftmotion_pa_reset_state()` — сброс накопленного offset, предыдущей скорости и EMA-фильтра.

### 5.7 Совместимость с другими системами

| Компонент | Совместимость | Примечание |
|-----------|--------------|------------|
| FT Motion | Обязателен | SPA работает только в связке с FT Motion |
| Input Shaping | Полная | SPA применяется до shaping/smoothing |
| Smoothing | Полная | Не влияет на работу сглаживания |
| M900 | Да | K, L, E параметры |
| G92 E0 | Да | Сброс состояния + сброс очереди PA |

---

## 6. История изменений

### v4.6 — текущая версия (рекомендуется)

| Дата | Изменение | Задача |
|------|-----------|--------|
| 2026-07-11 | **Кардинальное изменение алгоритма:** дифференциальная модель `offset[t] = offset[t-1] + K·ΔVe` заменена на **прямую пропорцию** `offset[t] = K·Ve_smooth` | v4.6 |
| 2026-07-11 | Удалена переменная `spa_ve_prev_q16` — хранение предыдущей Ve_smooth больше не требуется | v4.6 |
| 2026-07-11 | Удалён интегратор `spa_pa_offset_q16 += pa_delta_q16` — offset перезаписывается, а не накапливается | v4.6 |
| 2026-07-11 | Удалён расчёт `dve_q16` (ΔVe) — больше не нужен | v4.6 |
| 2026-07-11 | `SPA_SOFT_CLAMP` помечен как устаревший — с прямым расчётом offset windup интегратора невозможен | v4.6 |
| 2026-07-11 | Жёсткий clamp по `PA_MAX_P_MM` применяется к `K·Ve_smooth`, а не к накопленному offset | v4.6 |
| 2026-07-11 | SPA v4.6 математически **эквивалентен Klipper Pressure Advance** | v4.6 |

### v4.5-beta2 — архив

| Дата | Изменение | Задача |
|------|-----------|--------|
| 2026-07-11 | Исправлен баг: `spa_ema_alpha_q16` не инициализировался из `SPA_EMA_ALPHA` (всегда 0) | Аудит |
| 2026-07-11 | Исправлен баг: `pa_max_offset_q16` не инициализировался из `PA_MAX_P_MM` (всегда 0) | Аудит |
| 2026-07-11 | Исправлена гонка: K теперь читается из `current_block->pa_K_q16`, а не из глобального `ftmotion_pa_k_q16` | Аудит |
| 2026-07-11 | Удалён мёртвый код: `static block_t* last_block` в `calc_traj_point()` | Аудит |
| 2026-07-11 | Удалён дублирующийся блок SPA в M900.cpp, который вызывал `ftmotion_pa_set_k()` напрямую, минуя `extruder_advance_K[]` | Аудит |
| 2026-07-11 | Явная инициализация SPA K в `settings.cpp` при загрузке конфига | Аудит |
| 2026-07-01 | Добавлен EMA-фильтр низких частот для Ve (`SPA_EMA_ALPHA`) | Task 1 |
| 2026-07-01 | Добавлена функция `ftmotion_pa_set_ema_alpha()` (M900 E) | Task 1 |
| 2026-07-01 | Добавлено мягкое клиппирование (`SPA_SOFT_CLAMP`) | Task 2 |
| 2026-07-01 | Добавлен пиковый offset в телеметрию (`SPA_PEAK_TRACKING`) | Task 3 |
| 2026-07-01 | Добавлен динамический PA_MAX_P_MM (`M900 L<value>`) | Task 4 |
| 2026-07-01 | Добавлен Retract Decay (`SPA_RETRACT_DECAY`) | v4.5 |

### v4.4

| Дата | Изменение | Автор |
|------|-----------|-------|
| 2026-07-01 | Раскомментирован `#define PA_LOOKAHEAD` — исправлен мёртвый код алгоритма | Аудит |
| 2026-07-01 | `spa_pa_offset_q16`: int32_t → int64_t — защита от переполнения | Аудит |
| 2026-07-01 | Добавлена `pa_max_offset_q16` — клиппинг offset по `PA_MAX_P_MM` | Аудит |

---

*Документация составлена на основе исходного кода Marlin Firmware (ветка bugfix-2.1.x). Алгоритм SPA v4.6 реализует пропорциональную модель offset = K·Ve_smooth (эквивалент Klipper PA) с EMA-сглаживанием скорости, жёстким clamp по PA_MAX_P_MM и Retract Decay, работающую в фиксированном временном кадре FT Motion. Per-block K устраняет race condition при смене K во время движения.*
