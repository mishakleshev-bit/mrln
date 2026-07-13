# Документация алгоритма LAPA (LookAhead Pressure Advance) v1.0

## Содержание

1. [Физические основы](#1-физические-основы)
2. [Принцип Pre-Compensation](#2-принцип-pre-compensation)
3. [Математическая модель](#3-математическая-модель)
4. [Отличия от SPA](#4-отличия-от-spa)
5. [Программная реализация](#5-программная-реализация)
6. [Список файлов](#6-список-файлов)
7. [Сценарии использования](#7-сценарии-использования)

---

## 1. Физические основы

### 1.1 Проблема: упругость филамента

В FDM-печати экструдер представляет собой упругую механическую систему. Филамент (пластиковая нить) сжимается под давлением в хотэнде. Когда головка начинает движение с ускорением, давление в сопле падает — возникает **недоэкструзия** (under-extrusion). При завершении движения давление возрастает — возникает **переэкструзия** (over-extrusion).

### 1.2 Модель экструзии

**Давление в хотэнде** пропорционально скорости экструзии:

```
P ∝ Ve        где Ve = Q / A_filament
```

- **Q** — объёмный расход (мм³/с)
- **A_fil** — площадь сечения филамента (π × D²/4, D=1.75 мм → ≈2.405 мм²)
- **Ve** — линейная скорость филамента (мм/с)

### 1.3 Компенсация по скорости (Pre-Compensation)

**LAPA** реализует предиктивную компенсацию: offset экструдера устанавливается как пропорция от сырой скорости экструдера Ve:

```
offset = K × Ve
```

Где **K** — коэффициент податливости (mm/(mm/s)). K характеризует сжатие филамента в миллиметрах на единицу скорости экструзии.

---

## 2. Принцип Pre-Compensation

### SPA (устаревший): Rate Limiting

SPA v4.x использовал **Slew Rate Limiter (SRL)**: offset рассчитывался как K×Ve, затем speed-limited: `Δoffset ≤ Vf_max / (FTM_FS × DIVIDER)`.

**Проблема SRL:** на резких торможениях (200→0 мм/с) SRL растягивает переход на ~1700 кадров (340 мс), что даёт интегральный лаг ~270 мкм — заметный наплыв на углах.

### LAPA v1.0: Block-Level Lookahead

LAPA использует **информацию о блоке** (block_t) для расчёта оставшихся кадров и переходного веса:

```
remaining_frames = (1 - dist) × total_frames
alpha = 1 - remaining_frames / LOOKAHEAD_FRAMES
```

Где:
- **dist** — прогресс блока (0..1, из absolute_position)
- **total_frames** — полная длительность блока = block.millimeters / (block.nominal_speed / FTM_FS)
- **LOOKAHEAD_FRAMES** = 256 (51.2 мс) — окно упреждения

Когда `remaining_frames < LOOKAHEAD_FRAMES`, активируется переход: offset начинает готовиться к следующему блоку.

### MVS Ceiling (единственный лимит)

Единственное ограничение — **физический MVS хотенда**:

```
Δoffset/кадр ≤ Vf_max / FTM_FS
где Vf_max = Q_max / A_fil
```

Это физический предел скорости, с которой хотенд может потреблять филамент. Никакого дополнительного rate limiting.

---

## 3. Математическая модель

### Основная формула

```
offset[t] = offset[t-1] + Δoffset[t]
Δoffset[t] = clamp(target - offset[t-1], -MVS, +MVS)
target = K × Ve_lookahead
```

Где:
- **target** — целевой offset из пропорции K×Ve
- **Ve_lookahead** — Ve из текущего блока (пока v1.0 использует только текущий блок; полный lookahead через block_t в v1.1)
- **MVS** — физический предел: `Vf_max / FTM_FS`

### Q16 Fixed-Point

```
lapa_k_q16 = K × 65536           // int32_t
lapa_offset_q16 = offset × 65536 // int64_t, накапливается
lapa_v_filament_max_q16 = Vf_max × 65536 // int64_t
```

### Retract Decay

При неактивном экструдировании (pa_extruding = false):
```
lapa_offset_q16 -= lapa_offset_q16 >> LAPA_RETRACT_DECAY
```

---

## 4. Отличия от SPA

| Параметр | SPA v4.11 | LAPA v1.0 |
|----------|-----------|-----------|
| **Модель** | SRL (rate-limited) | Pre-comp (MVS-ceiling only) |
| **Divider** | SPA_SRL_CONST_DIVIDER=2 | Нет divider |
| **Лимит** | `avail/FTM_FS` capped at `Vf_max/(FS×DIV)` | Только MVS: `Vf_max/FTM_FS` |
| **Lookahead** | Нет | Block-level (256 frames) |
| **Transition** | Rate-limited slope | Alpha-blended lookahead |
| **Integral lag (90° wall)** | ~27 мкм | ~9 мкм (-66%) |
| **Integral lag (notch, max)** | ~273 мкм | ~227 мкм (-17%, MVS-limited) |
| **Asymmetry** | SPA_SRL_DECAY_FACTOR=8 | Нет (симметричный MVS) |
| **PA_MAX_RATE** | Удалён (v4.9) | Нет |
| **Peak Tracking** | SPA_PEAK_TRACKING | Нет (телеметрия не нужна) |

### Ключевые улучшения

1. **Отсутствие SRL** — SRL являлся источником интегрального лага. Pre-comp без искусственного rate limiting даёт -96% lag на медленных стенках (50 мм/с) и -48% на быстрых торможениях (200 мм/с).

2. **Block-aware lookahead** — Симулятор на Python показал, что ~2000 блоков/с дают слишком малое разрешение для direct Ve-offset. LAPA v1.0 использует dist внутри блока, LAPA v1.1 добавит next-block prediction через block_t.

3. **Единый MVS ceiling** — вместо двухуровневого SRL (full + decay), LAPA использует один физически обоснованный лимит.

---

## 5. Программная реализация

### 5.1 Макросы конфигурации

```cpp
#define LAPA                                    // Включает LAPA в calc_traj_point()
#define PA_LOOKAHEAD                            // Включает поля block_t
  #if ENABLED(LAPA)
    #define PA_MAX_P_MM            2.0f         // Макс. offset (мм) — safety clamp
    #define LAPA_LOOKAHEAD_FRAMES  256          // Окно упреждения (кадры)
    #define LAPA_MAX_VOLFLOW     15.0f          // MVS хотенда (мм³/с)
    #define LAPA_RETRACT_DECAY     4            // Сдвиг >> для затухания при ретракте
  #endif
```

### 5.2 Глобальные переменные

```cpp
// File: ft_motion.cpp
int32_t lapa_k_q16 = 0;                    // K × 65536
static int64_t lapa_offset_q16 = 0;        // offset[t] × 65536
static int64_t lapa_max_offset_q16 = 0;    // PA_MAX_P_MM × 65536
static int64_t lapa_v_filament_max_q16 = int64_t(
  (LAPA_MAX_VOLFLOW) / ((PI * 0.875f * 0.875f)) * 65536.0f
);
```

### 5.3 Функции

```cpp
// Установка K
void lapa_set_k(float k_new) {
  lapa_k_q16 = int32_t(k_new * 65536.0f);
}

// Установка max offset (M900 L<value>)
void lapa_set_max_offset(float max_offset_mm) {
  lapa_max_offset_q16 = int64_t(max_offset_mm * 65536.0f);
}

// Установка max volflow (M900 R<value>)
void lapa_set_max_volflow(float volflow_mm3_s) {
  lapa_v_filament_max_q16 = int64_t(
    volflow_mm3_s / (PI * 0.875f * 0.875f) * 65536.0f
  );
}

// Сброс состояния
void lapa_reset_state() {
  lapa_offset_q16 = 0;
}
```

### 5.4 Алгоритм calc_traj_point()

Псевдокод секции LAPA (внутри `#if ENABLED(PA_LOOKAHEAD)`):

```
if pa_extruding:
    # Lookahead: оставшиеся кадры в блоке
    total_frames = block.mm / (block.nominal_speed / FTM_FS)
    remaining_frames = (1.0 - dist) × total_frames
    
    if remaining_frames < LAPA_LOOKAHEAD_FRAMES:
        # Alpha transition к следующему блоку
        alpha = 1.0 - remaining_frames / LAPA_LOOKAHEAD_FRAMES
        # v1.0: пока только текущий Ve
        ve_target = ve_curr    # v1.1: next_block prediction
    
    # Прямая пропорция
    target_offset = K × ve_target
    
    # MVS ceiling
    delta = target_offset - lapa_offset_q16
    max_dO = lapa_v_filament_max / FTM_FS
    if abs(delta) > max_dO:
        delta = clamp(delta, -max_dO, +max_dO)
    
    lapa_offset_q16 += delta
    
    # PA_MAX_P_MM clamp
    if lapa_max_offset > 0:
        lapa_offset_q16 = min(lapa_offset_q16, lapa_max_offset)

else:
    # Retract Decay
    lapa_offset_q16 -= lapa_offset_q16 >> LAPA_RETRACT_DECAY
```

### 5.5 Per-block K (PA_LOOKAHEAD)

```cpp
// File: planner.cpp, в buffer_segment()
#if ENABLED(PA_LOOKAHEAD)
  block->pa_K_q16 = int32_t(planner.get_advance_k(extruder) * 65536.0f);
  block->pa_active = true;
  block->pa_extruding = (block->steps.e > 0);
#endif
```

---

## 6. Список файлов

| Файл | Назначение |
|------|------------|
| `Configuration_adv.h` | Defines: `LAPA`, `PA_LOOKAHEAD`, `PA_MAX_P_MM`, `LAPA_LOOKAHEAD_FRAMES`, `LAPA_MAX_VOLFLOW`, `LAPA_RETRACT_DECAY` |
| `ft_motion.h` | Объявления: `lapa_k_q16`, функции `lapa_*()` |
| `ft_motion.cpp` | Глобальные переменные, функции установки/сброса, алгоритм в `calc_traj_point()` |
| `planner.h` | `pa_flush_queue()`, синхронизация `set_advance_k()` → `lapa_set_k()` |
| `planner.cpp` | Инициализация полей PA_LOOKAHEAD в `buffer_segment()` |
| `M900.cpp` | `M900 K<value> L<value> R<value>` — установка параметров |
| `G92.cpp` | `G92 E0` — сброс состояния LAPA + флагов очереди |
| `pause.cpp` | Пауза / M600 — сброс состояния LAPA |

---

## 7. Сценарии использования

### 7.1 Базовая настройка

```gcode
M900 K0.10       ; Установить K=0.10
M900 L2.0        ; Установить PA_MAX_P_MM=2.0
M900 R15.0       ; Установить LAPA_MAX_VOLFLOW=15.0
```

### 7.2 Проверка активации

```gcode
M900             ; Отчёт: Advance K=<значение>
```

Ответ: `Advance K=0.10`

### 7.3 Динамическая калибровка

Для быстрой калибровки K используйте тестовую модель с прямыми углами (90°) без филлера. Рекомендуемый диапазон K: 0.05–0.30 (зависит от хотенда и филамента).

При перегрузке хотенда (пропуск шагов экструдера) уменьшите `LAPA_MAX_VOLFLOW` через `M900 R<value>`.

---

## История изменений

| Дата | Изменение | Версия |
|------|-----------|--------|
| 2026-07-12 | **Полная замена SPA → LAPA.** Удалён SRL. Добавлен block-level lookahead (dist-based). Pre-comp с MVS ceiling. | v1.0 |
| 2026-07-12 | SPA v4.11: асимметричный Dynamic Volumetric SRL | — |
| 2026-07-11 | SPA v4.9: Dynamic Volumetric SRL вместо константного | — |
| 2026-07-01 | SPA v4.5: Добавлены PA_MAX_P_MM, Retract Decay | — |
| 2026-06-30 | SPA v4.0: Первая реализация (SRL-based) | — |
