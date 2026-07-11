# Документация алгоритма SPA (Simplified Pressure Advance) v4.5

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

### 1.3 Упрощённый PA: дифференциальная модель (SPA v4.5)

**SPA (Simplified Pressure Advance)** реализует **дифференциальную модель**, основанную на изменении скорости экструзии между последовательными временными отсчётами (кадрами FT Motion):

```
ΔVe = Ve[текущий] - Ve[предыдущий]
E_компенсация += K × ΔVe
```

**Ключевые отличия от классического LinAdvance:**
- Использует EMA-фильтр низких частот для Ve (`SPA_EMA_ALPHA`) — Task 1
- Не требует знания ускорения блока
- Работает в фиксированном временном кадре `FTM_TS = 1 / FTM_FS`
- Компенсация накапливается (интегратор), а не вычисляется заново каждый кадр
- Минимальная задержка — один кадр (≈ 100–200 мкс)
- Мягкое клиппирование (`SPA_SOFT_CLAMP`) — Task 2
- Плавный сброс offset при ретрактах (`SPA_RETRACT_DECAY`)

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
| `ΔVe[t]` | Изменение сглаженной скорости | мм/с |
| `offset[t]` | Накопленная компенсация | мм |
| `offset_max` | Максимальный лимит offset (PA_MAX_P_MM) | мм |
| `FTM_FS` | Частота дискретизации FT Motion | Гц |
| `FTM_TS` | Период кадра = 1 / FTM_FS | с |
| `Q16` | Формат фиксированной точки: 16 дробных битов | — |

### 2.2 Уравнения (v4.5)

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

**Шаг 2. Вычисление изменения сглаженной скорости:**
```
ΔVe[t] = Ve_smooth[t] - Ve_smooth[t-1]                (2)
```

**Шаг 3. Вычисление приращения компенсации:**
```
Δoffset[t] = K × ΔVe[t]                                  (3)
```

**Шаг 4. Накопление компенсации (только при экструзии) с клиппингом:**
```
offset[t] = offset[t-1] + Δoffset[t]

// Мягкое клиппирование (SPA_SOFT_CLAMP):
if (offset[t] > offset_max) {
    excess = offset[t] - offset_max
    offset[t] = offset_max + excess / 2    // затухание превышения на 50%
}
```
Мягкое клиппирование заменяет жёсткий `clamp()`, который приводил к "ослеплению" интегратора: offset застревал на пределе, вызывая волнообразные артефакты (z-banding). При мягком клиппировании offset может кратковременно превышать лимит, но экспоненциально затухает к нему.

**Шаг 4a. Затухание при ретракте (Retract Decay):**
```
Если не па_extruding:
    offset[t] = offset[t-1] - offset[t-1] >> SPA_RETRACT_DECAY
```

**Шаг 5. Скорректированное положение экструдера:**
```
E_corrected[t] = E_planned[t] + offset[t]                (5)
```

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
Δoffset [мм] = K [с] × ΔVe [мм/с] = [мм] ✓
```

`K = 0.1` означает, что при изменении скорости экструзии на 10 мм/с компенсация составит 1 мм.

### 2.5 Q16 — формат фиксированной точки

Все вычисления SPA выполняются в целочисленной арифметике **Q16** (16 дробных битов).

```
Значение_Q16 = round(значение_float × 2^16) = round(значение_float × 65536)
```

**Используемые переменные в Q16:**

| Переменная | Тип | Описание | Q16 |
|-----------|-----|----------|-----|
| `ftmotion_pa_k_q16` | int32_t | K × 65536 | Q16 |
| `spa_ve_prev_q16` | int32_t | Ve_smooth[t-1] × 65536 | Q16 |
| `spa_ve_smooth_q16` | int32_t | Ve_smooth[t] × 65536 | Q16 |
| `spa_ema_alpha_q16` | int32_t | α × 65536 | Q16 |
| `spa_pa_offset_q16` | int64_t | offset[t] × 65536 | Q16 (хранится в int64_t) |
| `pa_max_offset_q16` | int64_t | offset_max × 65536 | Q16 (0 = без лимита) |

### 2.6 Поведение интегратора

SPA использует **накапливающийся** (интегрирующий) offset, что даёт:
- **Плавность**: компенсация не дёргается при кратковременных флуктуациях
- **Автоматический возврат к нулю**: после завершения движения ΔVe = 0 → offset перестаёт меняться
- **Устойчивость**: интегратор не накапливает ошибку, так как ΔVe — знакопеременная величина

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

Дополнительные макросы (v4.5):
- `SPA_EMA_ALPHA` — коэффициент EMA-фильтра (Task 1)
- `SPA_SOFT_CLAMP` — мягкое клиппирование (Task 2)
- `SPA_TELEMETRY` — вывод телеметрии
- `SPA_PEAK_TRACKING` — пиковый offset в телеметрии (Task 3)
- `SPA_RETRACT_DECAY` — затухание при ретрактах

### 3.3 Глобальные переменные

Определены в [`ft_motion.cpp`](Marlin/src/module/ft_motion.cpp:54):

```cpp
#if ENABLED(SIMPLIFIED_PA)
int32_t ftmotion_pa_k_q16 = 0;                  // K в Q16 (K_float × 65536)
static int32_t spa_ve_prev_q16 = 0;             // Предыдущая сглаженная скорость (Q16)
static int32_t spa_ve_smooth_q16 = 0;           // Сглаженная скорость (Q16) — EMA
static int64_t spa_pa_offset_q16 = 0;           // Накопленная компенсация (Q16 в int64_t)
static int64_t pa_max_offset_q16 = 0;           // Лимит offset (PA_MAX_P_MM × 65536), 0 = без лимита
int32_t spa_ema_alpha_q16 = 0;                  // Коэффициент EMA (Q16)

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
  pa_max_offset_q16 = int64_t(PA_MAX_P_MM * 65536.0f);
  spa_ema_alpha_q16 = int32_t(SPA_EMA_ALPHA * 65536.0f);  // Инициализация EMA
}
```

#### Установка лимита offset — [`ftmotion_pa_set_max_offset()`](Marlin/src/module/ft_motion.cpp:72) (Task 4)

```cpp
void ftmotion_pa_set_max_offset(float max_offset_mm) {
  pa_max_offset_q16 = int64_t(max_offset_mm * 65536.0f);
}
```

Управляется через `M900 L<value>`. Диапазон: 0.1–10.0 мм.

#### Установка EMA-альфа — [`ftmotion_pa_set_ema_alpha()`](Marlin/src/module/ft_motion.cpp:77) (Task 1)

```cpp
void ftmotion_pa_set_ema_alpha(float alpha) {
  LIMIT(alpha, 0.0f, 1.0f);
  spa_ema_alpha_q16 = int32_t(alpha * 65536.0f);
}
```

Управляется через `M900 E<alpha>`. Диапазон: 0.0–1.0.

#### Сброс состояния — [`ftmotion_pa_reset_state()`](Marlin/src/module/ft_motion.cpp:83)

```cpp
void ftmotion_pa_reset_state() {
  spa_ve_prev_q16 = 0;
  spa_ve_smooth_q16 = 0;
  spa_pa_offset_q16 = 0;
  #if ENABLED(SPA_PEAK_TRACKING)
    spa_peak_offset_q16 = 0;
  #endif
}
```

#### Основной алгоритм — [`calc_traj_point()`](Marlin/src/module/ft_motion.cpp:542)

```cpp
// Внутри FTMotion::calc_traj_point(), секция SIMPLIFIED_PA:

#if ENABLED(PA_LOOKAHEAD)
  #if ENABLED(SIMPLIFIED_PA)
    if (current_block && current_block->pa_active) {
      const float e_planned = traj_coords.e;

      // (1) Ve_raw в Q16
      const int32_t ve_curr_raw_q16 = int32_t((e_planned - prev_traj_e) * 65536.0f * FTM_FS);

      // (T1) EMA-фильтр: ve_smooth += alpha * (ve_raw - ve_smooth)
      const int32_t ve_diff_q16 = ve_curr_raw_q16 - spa_ve_smooth_q16;
      spa_ve_smooth_q16 += int32_t((int64_t(spa_ema_alpha_q16) * int64_t(ve_diff_q16)) >> 16);

      // ΔVe от сглаженной скорости
      const int32_t dve_q16 = spa_ve_smooth_q16 - spa_ve_prev_q16;

      // (3) Δoffset = K × ΔVe
      const int64_t pa_delta_q16 = ((int64_t)ftmotion_pa_k_q16 * (int64_t)dve_q16) >> 16;

      if (current_block->pa_extruding) {
        spa_pa_offset_q16 += pa_delta_q16;

        // (4) Клиппинг (жёсткий или мягкий)
        if (pa_max_offset_q16 > 0) {
          #if ENABLED(SPA_SOFT_CLAMP)
            // Мягкое клиппирование: затухание превышения
            if (spa_pa_offset_q16 > pa_max_offset_q16) {
              const int64_t excess_q16 = spa_pa_offset_q16 - pa_max_offset_q16;
              spa_pa_offset_q16 = pa_max_offset_q16 + (excess_q16 >> 1);
            }
            else if (spa_pa_offset_q16 < -pa_max_offset_q16) {
              const int64_t excess_q16 = -pa_max_offset_q16 - spa_pa_offset_q16;
              spa_pa_offset_q16 = -pa_max_offset_q16 - (excess_q16 >> 1);
            }
          #else
            // Жёсткий clamp (оригинальное поведение)
            if (spa_pa_offset_q16 > pa_max_offset_q16) ...
            if (spa_pa_offset_q16 < -pa_max_offset_q16) ...
          #endif
        }
      }
      else {
        // (4a) Retract Decay: затухание при ретракте
        #if SPA_RETRACT_DECAY > 0
          spa_pa_offset_q16 -= spa_pa_offset_q16 >> SPA_RETRACT_DECAY;
        #endif
      }

      // (5) Обновление prev и применение offset
      spa_ve_prev_q16 = spa_ve_smooth_q16;
      prev_traj_e = e_planned;
      traj_coords.e += (float)spa_pa_offset_q16 * (1.0f / 65536.0f);
    }
  #endif
#endif
```

### 3.5 Вспомогательные структуры данных

Определены в [`planner.h`](Marlin/src/module/planner.h:322):

```cpp
#if ENABLED(PA_LOOKAHEAD)
  int32_t pa_K_q16;           // K для этого блока (копия на момент запуска)
  bool pa_active;             // Флаг активности SPA для блока
  bool pa_extruding;          // Флаг: есть шаги экструдера (steps.e > 0)
#endif
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
| [`Configuration_adv.h`](Marlin/Configuration_adv.h:4904) | 4904–4940 | Defines: `SIMPLIFIED_PA`, `PA_LOOKAHEAD`, `SPA_EMA_ALPHA`, `SPA_SOFT_CLAMP`, `SPA_TELEMETRY`, `SPA_PEAK_TRACKING`, `PA_MAX_P_MM`, `SPA_RETRACT_DECAY` |

### 4.2 Ядро SPA

| Файл | Строки | Описание |
|------|--------|----------|
| [`ft_motion.h`](Marlin/src/module/ft_motion.h:433) | 433–445 | Объявления: `ftmotion_pa_k_q16`, `spa_ema_alpha_q16`, функции установки/чтения |
| [`ft_motion.cpp`](Marlin/src/module/ft_motion.cpp:54) | 54–95 | Глобальные переменные и функции установки/сброса |
| [`ft_motion.cpp`](Marlin/src/module/ft_motion.cpp:542) | 542–610 | Основной алгоритм в `calc_traj_point()` |

### 4.3 Планировщик (block_t)

| Файл | Строки | Описание |
|------|--------|----------|
| [`planner.h`](Marlin/src/module/planner.h:322) | 322–326 | Поля block_t: `pa_K_q16`, `pa_active`, `pa_extruding` |
| [`planner.h`](Marlin/src/module/planner.h:1166) | 1166–1172 | `pa_flush_queue()` — сброс флагов в очереди |
| [`planner.cpp`](Marlin/src/module/planner.cpp:2489) | 2489–2493 | Инициализация полей SPA в `buffer_segment()` |

### 4.4 Внешние вызовы

| Файл | Строки | Описание |
|------|--------|----------|
| [`M900.cpp`](Marlin/src/gcode/feature/advance/M900.cpp:122) | 122–150 | `M900 K<value> L<value> E<alpha>` — установка параметров |
| [`G92.cpp`](Marlin/src/gcode/geometry/G92.cpp:134) | 134–139 | `G92 E0` — сброс состояния SPA + флагов очереди |
| [`pause.cpp`](Marlin/src/feature/pause.cpp:399) | 399–402 | Пауза / M600 — сброс состояния SPA |
| [`stepper.h`](Marlin/src/module/stepper.h:635) | 635–637 | `get_current_block()` — доступ к текущему блоку |

---

## 5. Сценарии использования

### 5.1 Проверка активации SPA

1. Отправьте `M900` без параметров — ответ должен содержать `Advance K=<значение>`
2. При включённой `SPA_TELEMETRY` в терминале должны отображаться строки `PA|K=...`
3. При изменении K через `M900 K0.15` должен прийти ответ `SPA K set to 0.15 s`

### 5.2 Настройка K

Рекомендуемый метод: печать PA-башни с изменением K от 0.00 до 0.30.

Типичные значения: 0.05–0.30 в зависимости от вязкости пластика, длины хотэнда, скорости печати.

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

### v4.5 — текущая версия (аудит-доработка)

| Дата | Изменение | Задача |
|------|-----------|--------|
| 2026-07-11 | Добавлен EMA-фильтр низких частот для Ve (`SPA_EMA_ALPHA`) | Task 1 |
| 2026-07-11 | Добавлена переменная `spa_ve_smooth_q16` для сглаженной скорости | Task 1 |
| 2026-07-11 | Добавлена функция `ftmotion_pa_set_ema_alpha()` (M900 E) | Task 1 |
| 2026-07-11 | Добавлено мягкое клиппирование (`SPA_SOFT_CLAMP`) | Task 2 |
| 2026-07-11 | Мягкое клиппирование: экспоненциальное затухание превышения offset | Task 2 |
| 2026-07-11 | Добавлен пиковый offset в телеметрию (`SPA_PEAK_TRACKING`) | Task 3 |
| 2026-07-11 | Телеметрия выводит OffPeak и alpha | Task 3 |
| 2026-07-11 | Добавлен динамический PA_MAX_P_MM (`ftmotion_pa_set_max_offset()`, M900 L) | Task 4 |
| 2026-07-11 | Добавлен Retract Decay (`SPA_RETRACT_DECAY`) — плавный сброс offset при ретрактах | Аудит §3.2 |
| 2026-07-11 | `ftmotion_pa_reset_state()` теперь сбрасывает также `spa_ve_smooth_q16` и `spa_peak_offset_q16` | Сопровождение |

### v4.4

| Дата | Изменение | Автор |
|------|-----------|-------|
| 2026-07-11 | Раскомментирован `#define PA_LOOKAHEAD` — исправлен мёртвый код алгоритма | Аудит |
| 2026-07-11 | `spa_pa_offset_q16`: int32_t → int64_t — защита от переполнения | Аудит |
| 2026-07-11 | Добавлена `pa_max_offset_q16` — клиппинг offset по `PA_MAX_P_MM` | Аудит |
| 2026-07-11 | Инициализация лимита offset в `ftmotion_pa_set_k()` | Аудит |

---

*Документация составлена на основе исходного кода Marlin Firmware (ветка bugfix-2.1.x). Алгоритм SPA v4.5 реализует дифференциальную модель (K·ΔVe) с EMA-сглаживанием скорости, мягким клиппированием и динамическим лимитом offset, работающую в фиксированном временном кадре FT Motion.*
