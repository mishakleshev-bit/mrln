# Dual-Pipeline LA for FT Motion — Todo

## Phase 1: Cleanup
- [ ] 1.1 Удалить неиспользуемые файлы CONSTANT_JOLT:
  - `constant_jolt_math.h`
  - `constant_jolt_planner.cpp`
  - `constant_jolt_planner.h`
  - `trajectory_constant_jolt.cpp`
  - `trajectory_constant_jolt.h`

## Phase 2: Trajectory Generators — Acceleration Interface
- [ ] 2.1 Добавить `inline float getAccel(float t_norm, float delta, float duration)` в каждый генератор:
  - `TrajectoryTrapezoidal` — кусочно-постоянная a(t)
  - `Poly5TrajectoryGenerator` — closed-form 2nd derivative
  - `Poly6TrajectoryGenerator` — closed-form 2nd derivative
- [ ] 2.2 Добавить `float traj_duration` и `xyze_float_t traj_delta` в `FTMotion` (заполняются в `plan_next_block`)

## Phase 3: Dual Pipeline in calc_traj_point
- [ ] 3.1 Извлечь ускорение E из текущего генератора по `t_norm`
- [ ] 3.2 Заменить velocity-based LA на acceleration-based:
  ```cpp
  const float e_accel = currentGenerator->getAccel(t_norm, ratio.e * block->millimeters, block->millimeters / block->nominal_speed);
  traj_coords.e += e_accel * planner.get_advance_k();
  ```
- [ ] 3.3 Добавить `clamp()` для `extra_e` (защита от step overflow)
- [ ] 3.4 Исключить E из shaping pipeline: заменить `SHAPED_MAP(_SHAPE)` на `CARTES_MAP(_SHAPE)` в `calc_traj_point()`

## Phase 4: Phase Delay Compensation
- [ ] 4.1 Создать циклический буфер задержки для E на основе `ftm_zmax`
- [ ] 4.2 В `fill_stepper_plan_buffer`/`stepping` задерживать вывод E на `largest_delay_samples` фреймов
- [ ] 4.3 Обновить `ensure_extruder_float_precision` для работы с delay buffer
- [ ] 4.4 Обновить `calc_runout_samples` — E не добавляет runout

## Phase 5: Dynamic Frequency Safety
- [ ] 5.1 В `dynFreqMode_MASS_BASED` читать `traj_coords.e` **до** применения LA (снять LA extra перед dynFreq или сохранить raw value)

## Phase 6: Validation
- [ ] 6.1 Проверить отсутствие регрессий:
  - Без LA: поведение XYZ и E не изменилось
  - Без shaping: E не задерживается впустую
  - С LA + shaping: E выводится синхронно с XYZ
- [ ] 6.2 Проверить обработку retract (use_advance_lead == false)
