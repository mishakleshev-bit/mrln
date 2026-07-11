# SPA v4.5 → v4.6: Прямая пропорция offset = K × Ve_smooth

## Обоснование изменений

Текущий алгоритм использует **интегратор**: `offset[t] = offset[t-1] + K × dVe_smooth[t]`.  
Это вносит фазовую задержку, т.к. dVe вычисляется от EMA-сглаженной скорости.

Новый алгоритм использует **прямую пропорцию**: `offset[t] = K × Ve_smooth[t]`.  
Реакция мгновенная, нет windup, эквивалентно Klipper PA.

## Список изменяемых файлов

| № | Файл | Описание изменений |
|---|------|--------------------|
| 1 | `Marlin/src/module/ft_motion.cpp` (глобальные переменные) | Удалить `spa_ve_prev_q16`, сохранить остальные |
| 2 | `Marlin/src/module/ft_motion.cpp` (`ftmotion_pa_reset_state()`) | Убрать сброс `spa_ve_prev_q16` |
| 3 | `Marlin/src/module/ft_motion.cpp` (`calc_traj_point()`) | Заменить интегратор на прямую пропорцию |
| 4 | `Marlin/Configuration_adv.h` | Обновить комментарий SPA_SOFT_CLAMP |
| 5 | `docs/SPA_Documentation.md` | Обновить разделы 2.2, 3.4, 6 |

---

## Детальные спецификации изменений

### 1. [`ft_motion.cpp:54-93`](Marlin/src/module/ft_motion.cpp:54) — Глобальные переменные и функции

**Удалить:**
```cpp
static int32_t spa_ve_prev_q16 = 0;   // строка 57 — больше не нужен
```

**Изменить `ftmotion_pa_reset_state()` (строка 85):**
```cpp
void ftmotion_pa_reset_state() {
  spa_ve_smooth_q16 = 0;               // удалено: spa_ve_prev_q16 = 0;
  spa_pa_offset_q16 = 0;
  #if ENABLED(SPA_PEAK_TRACKING)
    spa_peak_offset_q16 = 0;
  #endif
}
```

### 2. [`ft_motion.cpp:537-607`](Marlin/src/module/ft_motion.cpp:537) — Алгоритм `calc_traj_point()`

**Было (интегратор + dVe от EMA):**
```cpp
// dVe от сглаженной скорости
const int32_t dve_q16 = spa_ve_smooth_q16 - spa_ve_prev_q16;

// Δoffset = K × dVe
const int64_t pa_delta_q16 = ((int64_t)block_K_q16 * (int64_t)dve_q16) >> 16;

if (current_block->pa_extruding) {
    spa_pa_offset_q16 += pa_delta_q16;           // интегратор

    // клиппинг (soft или hard)
    if (pa_max_offset_q16 > 0) {
        #if ENABLED(SPA_SOFT_CLAMP) ... #endif
    }
} else {
    #if SPA_RETRACT_DECAY > 0
        spa_pa_offset_q16 -= spa_pa_offset_q16 >> SPA_RETRACT_DECAY;
    #endif
}

spa_ve_prev_q16 = spa_ve_smooth_q16;   // обновление prev
```

**Стало (прямая пропорция):**
```cpp
// Прямая пропорция: offset = K × Ve_smooth (замена интегратора)
const int64_t pa_offset_new_q16 = ((int64_t)block_K_q16 * (int64_t)spa_ve_smooth_q16) >> 16;

if (current_block->pa_extruding) {
    // Клиппинг (жёсткий — soft clamp больше не нужен, т.к. нет windup)
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
    // Retract Decay: плавный сброс offset при ретрактах
    #if SPA_RETRACT_DECAY > 0
        spa_pa_offset_q16 -= spa_pa_offset_q16 >> SPA_RETRACT_DECAY;
    #endif
}

// Применение offset
traj_coords.e += (float)spa_pa_offset_q16 * (1.0f / 65536.0f);

// Обновление prev_traj_e (для Ve_raw в следующем кадре)
prev_traj_e = e_planned;

// spa_ve_prev_q16 удалён — больше не нужен
```

**Важно:** `spa_ve_smooth_q16` **обновляется ДО** вычисления offset:
```cpp
// (T1) EMA-фильтр — ЭТО ОСТАЁТСЯ без изменений
spa_ve_smooth_q16 += int32_t((int64_t(spa_ema_alpha_q16) * int64_t(ve_diff_q16)) >> 16);

// затем сразу offset = K × Ve_smooth
const int64_t pa_offset_new_q16 = ((int64_t)block_K_q16 * (int64_t)spa_ve_smooth_q16) >> 16;
```

**Порядок вычислений в новом алгоритме:**
```
1. ve_curr_raw_q16 = (e_planned - prev_traj_e) × FTM_FS       // Ve_raw
2. spa_ve_smooth_q16 += α × (ve_curr_raw - spa_ve_smooth)     // EMA-фильтр
3. offset_new = K × spa_ve_smooth_q16                          // ПРЯМАЯ ПРОПОРЦИЯ
4. clip(offset_new)                                            // Клиппинг
5. traj_coords.e += offset                                     // Применение
6. prev_traj_e = e_planned                                     // На следующий кадр
```

### 3. [`Configuration_adv.h:4936-4939`](Marlin/Configuration_adv.h:4936) — Конфигурация

```cpp
// ──────────────────────────────────────────────────────────────────────────
// Task 2: Мягкое клиппирование (Soft Saturation / Anti-Windup)
// ──────────────────────────────────────────────────────────────────────────
// В v4.6+ с переходом на прямую пропорцию offset = K × Ve_smooth
// windup интегратора устранён, поэтому SPA_SOFT_CLAMP больше не актуален.
// Используется жёсткий clamp по PA_MAX_P_MM.
//#define SPA_SOFT_CLAMP
```

Макрос можно закомментировать и пометить как неактуальный, либо оставить для обратной совместимости — клиппинг всё равно жёсткий.

### 4. [`docs/SPA_Documentation.md`](docs/SPA_Documentation.md) — Документация

- **Раздел 2.2** — заменить уравнения (3)-(5) на `offset = K × Ve_smooth`
- **Раздел 3.4** — обновить псевдокод `calc_traj_point()`
- **Раздел 3.5** — удалить упоминание `spa_ve_prev_q16`
- **Раздел 6** — добавить запись в историю изменений

---

## ПОРЯДОК РЕАЛИЗАЦИИ

### Шаг 1: Изменить глобальные переменные
**Файл:** [`ft_motion.cpp:54-65`](Marlin/src/module/ft_motion.cpp:54)
- Удалить строку 57: `static int32_t spa_ve_prev_q16 = 0;`

### Шаг 2: Изменить `ftmotion_pa_reset_state()`
**Файл:** [`ft_motion.cpp:85-92`](Marlin/src/module/ft_motion.cpp:85)
- Убрать `spa_ve_prev_q16 = 0;`

### Шаг 3: Изменить алгоритм в `calc_traj_point()`
**Файл:** [`ft_motion.cpp:542-607`](Marlin/src/module/ft_motion.cpp:542)
- Заменить секцию SPA:
  - Удалить `dve_q16` (строка 563)
  - Удалить `pa_delta_q16` (строка 567)
  - Удалить интегратор `spa_pa_offset_q16 += pa_delta_q16` (строка 571)
  - Удалить `spa_ve_prev_q16 = spa_ve_smooth_q16` (строка 602)
  - Добавить прямую пропорцию: `pa_offset_new_q16 = block_K_q16 * spa_ve_smooth_q16 >> 16`

### Шаг 4: Обновить конфигурацию
**Файл:** [`Configuration_adv.h:4936-4939`](Marlin/Configuration_adv.h:4936)
- Закомментировать `#define SPA_SOFT_CLAMP` и добавить пояснение

### Шаг 5: Обновить документацию
**Файл:** [`docs/SPA_Documentation.md`](docs/SPA_Documentation.md)
- Актуализировать описание алгоритма
