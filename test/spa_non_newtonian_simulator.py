#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LAPA Non-Newtonian FDM Simulator v3.0
======================================
Физическая симуляция течения расплава полимера в сопле FDM-принтера.
Сравнивает:
  (A) Linear PA:        offset = K * Ve
  (B) Power-Law PA:     offset = C * Ve^n
  (C) Adaptive MVS:     Vf_max_eff(Ve) = Vf_max * (1 + α * (Ve/Ve_ref)^β)
  (D) Power-Law + Adaptive MVS: комбинация B + C
  + Block-based simulation с lookahead pre-comp (LAPA)

Зависимости: numpy, matplotlib
  pip install numpy matplotlib
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')  # для сохранения без дисплея
import matplotlib.pyplot as plt
from dataclasses import dataclass, field
from typing import List, Tuple, Dict


# ═══════════════════════════════════════════════════════════════════════
# 1. Параметры физики
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class NozzleConfig:
    """Геометрия сопла и параметры печати"""
    # Геометрия
    D_nozzle: float = 0.4        # [mm] диаметр отверстия сопла
    D_filament: float = 1.75     # [mm] диаметр филамента
    L_nozzle: float = 2.0        # [mm] длина канала сопла

    # Реология (PETG при 240°C)
    n_power: float = 0.60        # [-] показатель степени (0.5-0.7 для PETG)
    K_consistency: float = 1500  # [Pa·s^n] консистентность

    # PA
    K_pa: float = 0.05           # [s] коэффициент Pressure Advance (при 50mm/s)

    # Adaptive MVS
    alpha_mvs: float = 0.25      # [-] макс. увеличение Vf_max (25% на макс. скорости)
    beta_mvs: float = 0.50       # [-] степень: Vf_max_eff ∝ Ve^beta (0.5 = sqrt)
    ve_ref_mvs: float = 1.829    # [mm/s] Ve при XY=50mm/s (точка привязки)

    @property
    def R_nozzle(self) -> float:
        return self.D_nozzle / 2

    @property
    def R_fil(self) -> float:
        return self.D_filament / 2

    @property
    def A_nozzle(self) -> float:
        return np.pi * self.R_nozzle ** 2

    @property
    def A_fil(self) -> float:
        return np.pi * self.R_fil ** 2


@dataclass
class MotionProfile:
    """Профиль движения — имитация угла на тестовой детали"""
    name: str
    v_xy_before: float    # [mm/s] скорость до угла
    v_xy_after: float     # [mm/s] скорость после угла
    angle: float          # [deg] угол поворота
    v_xy_approach: float = 0.0  # [mm/s] скорость подхода
    jd: float = 0.0       # [mm] Junction Deviation (0 = не используется)

    def calc_junction_speed(self, accel: float = 3000.0) -> float:
        """
        Расчёт скорости на углу через Junction Deviation.

        v_junction = sqrt(JD × accel × (1 - cos(θ)) / sin(θ))

        Для θ=90°, sin=1.0, cos=0 → v_junct = sqrt(JD * accel)
        Для JD=0.03: v = sqrt(90) = 9.5mm/s
        Для JD=0.06: v = sqrt(180) = 13.4mm/s
        """
        if self.jd <= 0:
            return self.v_xy_after
        theta_rad = np.radians(self.angle)
        v_j = np.sqrt(self.jd * accel * (1.0 - np.cos(theta_rad)) / np.sin(theta_rad))
        return float(v_j)

    def calc_ve(self, cfg: NozzleConfig, layer_height: float = 0.2,
                extrusion_width: float = 0.44) -> Tuple[float, float]:
        """Расчёт Ve из XY скорости"""
        q_before = self.v_xy_before * layer_height * extrusion_width
        if self.jd > 0:
            # Используем junction speed вместо v_xy_after
            v_j = self.calc_junction_speed()
            q_after = v_j * layer_height * extrusion_width
        else:
            q_after = self.v_xy_after * layer_height * extrusion_width
        ve_before = q_before / cfg.A_fil
        ve_after = q_after / cfg.A_fil

        if self.v_xy_approach > 0:
            q_approach = self.v_xy_approach * layer_height * extrusion_width
            self.ve_approach = q_approach / cfg.A_fil
        else:
            self.ve_approach = ve_before

        return ve_before, ve_after


# ═══════════════════════════════════════════════════════════════════════
# 2. LAPA Block Model
# ═══════════════════════════════════════════════════════════════════════

@dataclass
class Block:
    """Один блок движения (аналог block_t в Marlin)"""
    name: str
    v_xy: float          # [mm/s] номинальная XY скорость
    ve: float            # [mm/s] скорость экструзии (константа)
    length_mm: float     # [mm] длина блока
    accel: float = 3000  # [mm/s²] ускорение (для ramp)
    ve_start: float = None  # Ve на старте (для trapezoid decel)
    ve_end: float = None    # Ve на финише (для trapezoid accel)

    @property
    def total_frames(self) -> int:
        """Число кадров в блоке при FTM_FS=5000"""
        if self.v_xy <= 0: return 0
        return max(1, int(self.length_mm / (self.v_xy / 5000.0)))

    def get_ve_at_dist(self, dist: float) -> float:
        """Ve на позиции dist=0..1 внутри блока"""
        if self.ve_start is not None and self.ve_end is not None:
            return self.ve_start + (self.ve_end - self.ve_start) * dist
        return self.ve


@dataclass
class BlockSequence:
    """Последовательность блоков — имитация реального G-code"""
    name: str
    blocks: List[Block]

    @property
    def total_frames(self) -> int:
        return sum(b.total_frames for b in self.blocks)

    def get_block_and_frame(self, global_frame: int) -> Tuple[int, int, float]:
        """
        По глобальному кадру возвращает:
        (block_idx, frame_in_block, dist)
        """
        remaining = global_frame
        for idx, b in enumerate(self.blocks):
            if remaining < b.total_frames:
                dist = remaining / max(b.total_frames, 1)
                return idx, remaining, dist
            remaining -= b.total_frames
        return len(self.blocks)-1, self.blocks[-1].total_frames-1, 1.0


# ═══════════════════════════════════════════════════════════════════════
# 3. Физическая модель неньютоновского течения
# ═══════════════════════════════════════════════════════════════════════

class NonNewtonianModel:
    """
    Модель неньютоновского течения в сопле.

    Степенной закон (Ostwald-de Waele):
        μ_app = K_cons × γ̇^(n-1)
        ΔP = 2K_cons L/R × [(3n+1)/n × Q/(πR³)]^n

    + Диссипативный разогрев (shear heating):
        ΔT_diss = ΔP / (ρ * Cp)
        μ_eff(T) = μ_app(T_ref) * exp[-b * (1/T - 1/T_ref)]
    """

    # Термофизические константы PETG
    RHO_PETG: float = 1200.0      # [kg/m³] плотность
    CP_PETG: float = 1500.0       # [J/(kg·K)] теплоёмкость
    B_ACT: float = 6500.0         # [K] энергия активации (Arrhenius)
    T_REF: float = 513.15         # [K] 240°C

    def __init__(self, cfg: NozzleConfig):
        self.cfg = cfg

    def shear_rate_nozzle(self, Q: float) -> float:
        """Скорость сдвига в сопле [с^-1]"""
        R = self.cfg.R_nozzle / 1000  # м
        Q_m3s = Q / 1e9               # мм³/с → м³/с
        return 4 * Q_m3s / (np.pi * R ** 3) if R > 0 else 0

    def apparent_viscosity(self, gamma_dot: float, T: float = None) -> float:
        """
        Кажущаяся вязкость [Pa·s] с температурной поправкой.

        μ(T) = K * γ̇^(n-1) * exp[b * (1/T - 1/T_ref)]
        """
        K = self.cfg.K_consistency
        n = self.cfg.n_power
        mu = K * gamma_dot ** (n - 1) if gamma_dot > 0 else 0

        if T is not None and T != self.T_REF:
            # Arrhenius-поправка: exp[b * (1/T - 1/T_ref)]
            # При T > T_ref: отрицательный показатель → μ снижается
            mu *= np.exp(self.B_ACT * (1.0 / T - 1.0 / self.T_REF))

        return mu

    def dissipative_heating(self, Q: float, delta_p: float) -> float:
        """
        Диссипативный разогрев расплава [K].

        ΔT_diss = ΔP / (ρ * Cp)
        (вся механическая работа → тепло)
        """
        if Q <= 0:
            return 0
        return delta_p / (self.RHO_PETG * self.CP_PETG)

    def pressure_drop(self, Q: float, T: float = None) -> float:
        """
        Перепад давления на сопле [Pa].

        ΔP = 2K(T)L/R × [(3n+1)/n × Q/(πR³)]^n

        С температурной поправкой через K_eff(T).
        """
        n = self.cfg.n_power
        K = self.cfg.K_consistency
        R = self.cfg.R_nozzle / 1000
        L = self.cfg.L_nozzle / 1000
        Q_m3s = Q / 1e9

        if R <= 0 or Q_m3s <= 0:
            return 0

        # Температурная поправка к K_consistency
        K_eff = K
        if T is not None and T != self.T_REF:
            K_eff *= np.exp(self.B_ACT * (1.0 / T - 1.0 / self.T_REF))

        factor = 2 * K_eff * L / R
        inner = ((3 * n + 1) / n) * Q_m3s / (np.pi * R ** 3)

        return factor * inner ** n

    def calc_flow_temp(self, Q: float, delta_p: float) -> float:
        """Итеративный расчёт температуры с учётом разогрева"""
        T = self.T_REF  # начальная температура
        # 2-3 итерации достаточно для сходимости
        for _ in range(3):
            dT = self.dissipative_heating(Q, delta_p)
            T_new = self.T_REF + dT
            # Пересчёт ΔP при новой температуре
            delta_p = self.pressure_drop(Q, T_new)
            T = T_new
        return T


# ═══════════════════════════════════════════════════════════════════════
# 3. SPA-симулятор (4 режима)
# ═══════════════════════════════════════════════════════════════════════

class SPA_Simulator:
    """
    Симуляция SPA на заданном профиле скорости.
    Поддерживает 6 режимов:
      - mode='linear':        offset = K * Ve,  Vf_max = const (v4.11 baseline)
      - mode='power_law':     offset = C * Ve^n, Vf_max = const
      - mode='adaptive_mvs':  offset = K * Ve,  Vf_max_eff(Ve) растёт
      - mode='full':          offset = C * Ve^n, Vf_max_eff(Ve) растёт
      - mode='lookahead':     offset = K * ve_lookahead(Ve_curr, Ve_future, remaining_frames)
                              Pre-compensation с SRL: начинает менять offset за
                              LOOKAHEAD_FRAMES кадров до события, но SRL всё ещё
                              rate-limiting (0.624μm/frame).
      - mode='precomp':       Pre-compensation без SRL (Klipper-like).
                              За LOOKAHEAD_FRAMES до события плавно доводит offset
                              до целевого значения. SRL отключён — offset назначается
                              напрямую. dO/dt ограничено только MVS ceiling.
    """

    def __init__(self, cfg: NozzleConfig, ftm_fs: int = 5000, lookahead_frames: int = 126):
        self.cfg = cfg
        self.ftm_fs = ftm_fs
        self.nn = NonNewtonianModel(cfg)
        self.lookahead_frames = lookahead_frames  # кадров lookahead (126 = 25.2ms)

    def vf_max_effective(self, ve_curr: float) -> float:
        """
        Adaptive MVS: Vf_max_eff(Ve) = Vf_max * (1 + α * (Ve/Ve_ref)^β)

        Vf_max (базовый): 6.24 mm/s = 15mm³/s / (π * (1.75/2)²)
        α = 0.25 → до +25% на макс. скорости
        β = 0.50 → квадратичный корень (быстрый рост вначале, насыщение)
        """
        vf_max_base = 6.24  # mm/s filament speed
        alpha = self.cfg.alpha_mvs
        beta = self.cfg.beta_mvs
        ve_ref = self.cfg.ve_ref_mvs

        if alpha <= 0 or ve_curr <= 0:
            return vf_max_base

        # Нормированное увеличение
        ratio = ve_curr / ve_ref
        boost = alpha * (ratio ** beta)
        # Ограничение: не более 2× базового
        boost = min(boost, 1.0)

        return vf_max_base * (1.0 + boost)

    def simulate(self, profile: MotionProfile, duration: float = 0.3,
                 mode: str = 'linear') -> dict:
        """
        Симуляция профиля движения.

        Parameters:
            mode: 'linear' | 'power_law' | 'adaptive_mvs' | 'full' |
                  'lookahead' | 'precomp'

        Returns:
            dict с массивами: t, ve, ve_smooth, offset, offset_target,
                              vf_max_eff, delta_p, temp_rise, q_vol
        """
        n_frames = int(duration * self.ftm_fs)
        dt = 1.0 / self.ftm_fs

        t = np.zeros(n_frames)
        ve = np.zeros(n_frames)
        ve_smooth = np.zeros(n_frames)
        offset = np.zeros(n_frames)
        offset_target = np.zeros(n_frames)
        vf_max_eff_arr = np.zeros(n_frames)
        delta_p = np.zeros(n_frames)
        temp_rise = np.zeros(n_frames)
        q_vol = np.zeros(n_frames)
        mvs_violation = np.zeros(n_frames, dtype=bool)

        # Флаги
        use_power_law = (mode in ('power_law', 'full'))
        use_adaptive_mvs = (mode in ('adaptive_mvs', 'full'))
        use_lookahead = (mode == 'lookahead')
        use_precomp = (mode == 'precomp')

        # Параметры SRL (v4.11: Symmetric Constant-Rate)
        srl_divider = 2

        # Power-law константа (привязка к K_pa при 50mm/s = ve_ref)
        if use_power_law:
            ve_ref = self.cfg.ve_ref_mvs
            n_eff = self.cfg.n_power
            C_power = self.cfg.K_pa * (ve_ref ** (1.0 - n_eff))
        else:
            C_power = 0  # не используется

        # Профиль Ve: ступенька со сглаженным переходом
        midpoint = n_frames // 2
        transition = 10  # кадров на переход

        for i in range(n_frames):
            if i < midpoint - transition:
                ve[i] = profile.ve_before
            elif i >= midpoint + transition:
                ve[i] = profile.ve_after
            else:
                # Sigmoid-переход
                frac = (i - (midpoint - transition)) / (2 * transition)
                sig = 1.0 / (1.0 + np.exp(-12 * (frac - 0.5)))
                ve[i] = profile.ve_before * (1 - sig) + profile.ve_after * sig

        # Дополнительное сглаживание Ve (имитация FT Motion)
        window = 3
        ve_smooth[:window] = ve[:window]
        for i in range(window, n_frames):
            ve_smooth[i] = ve_smooth[i-1] + (ve[i] - ve_smooth[i-1]) * 0.4

        # Основной цикл SPA
        for i in range(n_frames):
            ve_curr = ve_smooth[i]

            # (1) Vf_max_effective (может зависеть от Ve)
            if use_adaptive_mvs:
                vf_max = self.vf_max_effective(ve_curr)
            else:
                vf_max = 6.24  # базовый Vf_max
            vf_max_eff_arr[i] = vf_max

            # (2) Целевой offset
            if use_power_law:
                # Power-law: offset = C * Ve^n
                if ve_curr > 0:
                    offset_target_val = C_power * (ve_curr ** self.cfg.n_power)
                else:
                    offset_target_val = 0
            elif use_lookahead or use_precomp:
                # PA Lookahead / Pre-comp: offset = K * ve_lookahead
                # ve_lookahead = lerp(ve_curr, ve_future, alpha)
                # где alpha растёт от 0 до 1 за LOOKAHEAD_FRAMES до события
                ve_future = profile.ve_after
                frames_to_event = midpoint - i
                if frames_to_event > 0 and frames_to_event <= self.lookahead_frames:
                    alpha = 1.0 - float(frames_to_event) / float(self.lookahead_frames)
                    alpha = max(0.0, min(1.0, alpha))
                    ve_lookahead = ve_future * alpha + ve_curr * (1.0 - alpha)
                    offset_target_val = self.cfg.K_pa * ve_lookahead
                elif frames_to_event <= 0:
                    # После события — обычный расчёт
                    offset_target_val = self.cfg.K_pa * ve_curr
                else:
                    # Далеко до события — обычный расчёт
                    offset_target_val = self.cfg.K_pa * ve_curr
            else:
                # Linear: offset = K * Ve
                offset_target_val = self.cfg.K_pa * ve_curr

            offset_target[i] = offset_target_val

            # (3) Применение offset
            if use_precomp:
                # Pre-compensation (Klipper-like): offset назначается напрямую,
                # SRL отключён. MVS ceiling применяется как safety check.
                if i > 0:
                    dO = offset_target_val - offset[i-1]
                    max_dO = vf_max / self.ftm_fs
                    if abs(dO) > max_dO:
                        dO = np.clip(dO, -max_dO, max_dO)
                        mvs_violation[i] = True
                    offset[i] = offset[i-1] + dO
                else:
                    offset[i] = offset_target_val
            else:
                # SRL (v4.11: Symmetric Constant-Rate) — для всех остальных
                if i > 0:
                    delta = offset_target_val - offset[i-1]
                    c_limit = vf_max / (self.ftm_fs * srl_divider)
                    avail = vf_max - abs(ve_curr)
                    if avail > 0:
                        mvs_limit = avail / self.ftm_fs
                    else:
                        mvs_limit = vf_max / self.ftm_fs
                    limit = min(c_limit, mvs_limit)
                    delta = np.clip(delta, -limit, limit)
                    offset[i] = offset[i-1] + delta
                else:
                    offset[i] = offset_target_val

            # (4) Суммарный Ve с PA
            if i > 0:
                dO_dt = (offset[i] - offset[i-1]) / dt
            else:
                dO_dt = 0
            ve_total = max(0, ve_curr + dO_dt)
            q_vol[i] = ve_total * self.cfg.A_fil

            # (5) Давление с температурной поправкой (диссипативный разогрев)
            delta_p_i = self.nn.pressure_drop(q_vol[i])
            T_i = self.nn.calc_flow_temp(q_vol[i], delta_p_i)
            temp_rise[i] = T_i - self.nn.T_REF
            # Пересчёт ΔP при реальной температуре
            delta_p[i] = self.nn.pressure_drop(q_vol[i], T_i)

            t[i] = i * dt

        mvs_violations = int(np.sum(mvs_violation))

        return {'t': t, 've': ve, 've_smooth': ve_smooth,
                'offset': offset, 'offset_target': offset_target,
                'vf_max_eff': vf_max_eff_arr,
                'delta_p': delta_p, 'temp_rise': temp_rise, 'q_vol': q_vol,
                'mvs_violations': mvs_violations}


# ═══════════════════════════════════════════════════════════════════════
# 3b. Block-based simulator
# ═══════════════════════════════════════════════════════════════════════

def simulate_blocks(seq: BlockSequence, cfg: NozzleConfig,
                    mode: str = 'linear', lookahead_frames: int = 126,
                    ftm_fs: int = 5000, srl_divider: int = 2) -> dict:
    """
    Симуляция последовательности блоков с lookahead.

    Режимы (mode):
      'linear' — SRL v4.11 (rate-limited)
      'precomp' — LAPA v1.0 (pre-comp, MVS ceiling only, простой lookahead)
      'adaptive_lookahead' — LAPA v1.1 (true next-block prediction + adaptive window)

    Для каждого кадра:
      - dist = прогресс в текущем блоке
      - remaining_frames = (1-dist) * block.total_frames
      - Если remaining_frames < lookahead_frames и есть next_block:
          ve_future = next_block.ve
          alpha = 1 - remaining_frames / lookahead_frames
          ve_target = lerp(ve_curr, ve_future, alpha)
      - offset = K * ve_target
      - SRL или pre-comp в зависимости от mode
    """
    nn = NonNewtonianModel(cfg)
    n_frames = seq.total_frames
    # + дополнительное время для наблюдения после последнего блока
    n_frames += int(0.05 * ftm_fs)  # +50ms

    dt = 1.0 / ftm_fs

    t = np.zeros(n_frames)
    ve = np.zeros(n_frames)
    ve_smooth = np.zeros(n_frames)
    offset = np.zeros(n_frames)
    offset_target = np.zeros(n_frames)
    vf_max_eff_arr = np.zeros(n_frames)
    delta_p_arr = np.zeros(n_frames)
    temp_rise_arr = np.zeros(n_frames)
    q_vol_arr = np.zeros(n_frames)
    block_boundaries = []  # кадры стыков блоков

    use_precomp = (mode == 'precomp') or (mode == 'adaptive_lookahead')
    use_lookahead = (mode == 'lookahead')
    use_adaptive = (mode == 'adaptive_lookahead')
    vf_max_base = 6.24  # V_f_max для 1.75mm filament, 15mm³/s MVS
    min_lookahead_frames = 256  # LAPA_MIN_LOOKAHEAD

    for i in range(n_frames):
        block_idx, frame_in_block, dist = seq.get_block_and_frame(i)
        block = seq.blocks[min(block_idx, len(seq.blocks)-1)]

        # Отметка стыка
        if frame_in_block == 0 and i > 0:
            block_boundaries.append(i)

        # (1) ve_curr из траектории
        ve_curr = block.get_ve_at_dist(dist)

        # (2) remaining_frames
        remaining_frames = (1.0 - min(dist, 0.999)) * block.total_frames

        # (3) ve_target с lookahead / предкомпенсацией
        has_next = (block_idx + 1 < len(seq.blocks))
        effective_lookahead = lookahead_frames  # default fallback
        ve_target = ve_curr

        if remaining_frames < lookahead_frames and has_next:
            next_block_obj = seq.blocks[block_idx + 1]
            ve_future = next_block_obj.get_ve_at_dist(0.0)

            if use_adaptive:
                # LAPA v1.1: adaptive window based on required frames
                delta_ve = abs(ve_curr - ve_future)
                target_offset_delta = cfg.K_pa * delta_ve
                mvs_per_frame = vf_max_base / ftm_fs
                if mvs_per_frame > 1e-9:
                    required_frames = int(target_offset_delta / mvs_per_frame + 0.5)
                    effective_lookahead = max(required_frames, min_lookahead_frames)
                else:
                    effective_lookahead = lookahead_frames
            else:
                effective_lookahead = lookahead_frames

            # Alpha blend
            alpha = 1.0 - remaining_frames / max(effective_lookahead, 1)
            alpha = max(0.0, min(1.0, alpha))
            ve_target = ve_future * alpha + ve_curr * (1.0 - alpha)

        elif use_adaptive and remaining_frames < lookahead_frames and not has_next:
            # LAPA v1.1: decay-to-zero для последнего блока
            decay_alpha = min(1.0, remaining_frames / max(lookahead_frames, 1))
            ve_target = ve_curr * decay_alpha  # → 0 к концу

        # (4) Target offset
        offset_target_val = cfg.K_pa * ve_target
        offset_target[i] = offset_target_val

        # (5) Применение offset
        if use_precomp:
            # Pre-compensation: offset назначается напрямую, MVS safety check
            if i > 0:
                dO = offset_target_val - offset[i-1]
                max_dO = vf_max_base / ftm_fs
                if abs(dO) > max_dO:
                    dO = np.clip(dO, -max_dO, max_dO)
                offset[i] = offset[i-1] + dO
            else:
                offset[i] = offset_target_val
        else:
            # SRL для всех остальных режимов
            if i > 0:
                delta = offset_target_val - offset[i-1]
                c_limit = vf_max_base / (ftm_fs * srl_divider)
                avail = vf_max_base - abs(ve_curr)
                mvs_limit = (avail / ftm_fs) if avail > 0 else vf_max_base / ftm_fs
                limit = min(c_limit, mvs_limit)
                delta = np.clip(delta, -limit, limit)
                offset[i] = offset[i-1] + delta
            else:
                offset[i] = offset_target_val

        # (6) Суммарный Ve с PA
        if i > 0:
            dO_dt = (offset[i] - offset[i-1]) / dt
        else:
            dO_dt = 0
        ve_total = max(0, ve_curr + dO_dt)
        q_vol_arr[i] = ve_total * cfg.A_fil

        # (7) Давление
        delta_p_i = nn.pressure_drop(q_vol_arr[i])
        T_i = nn.calc_flow_temp(q_vol_arr[i], delta_p_i)
        temp_rise_arr[i] = T_i - nn.T_REF
        delta_p_arr[i] = nn.pressure_drop(q_vol_arr[i], T_i)

        t[i] = i * dt
        vf_max_eff_arr[i] = vf_max_base

    return {'t': t, 've': ve, 've_smooth': ve,
            'offset': offset, 'offset_target': offset_target,
            'vf_max_eff': vf_max_eff_arr,
            'delta_p': delta_p_arr, 'temp_rise': temp_rise_arr, 'q_vol': q_vol_arr,
            'block_boundaries': block_boundaries, 'mvs_violations': 0}


# ═══════════════════════════════════════════════════════════════════════
# 4. Анализ lag
# ═══════════════════════════════════════════════════════════════════════

def calc_lag(result: dict) -> dict:
    """Метрики lag: offset vs target"""
    offset = result['offset']
    target = result['offset_target']
    lag_abs = np.abs(target - offset)
    lag_pct = np.where(np.abs(target) > 1e-8, lag_abs / np.abs(target) * 100, 0)
    integral = np.trapezoid(lag_abs, result['t']) if len(result['t']) > 1 else 0

    return {
        'lag_abs': lag_abs,
        'lag_pct': lag_pct,
        'max_lag_abs': float(np.max(lag_abs)),
        'max_lag_pct': float(np.max(lag_pct)),
        'integral_lag': float(integral)
    }


# ═══════════════════════════════════════════════════════════════════════
# 5. Графики
# ═══════════════════════════════════════════════════════════════════════

def plot_comparison(results: Dict[str, dict], titles: List[str], filename: str):
    """Сравнение 4 режимов: offset, Ve, ΔP, lag, Vf_max_eff, temp_rise"""
    fig, axes = plt.subplots(3, 3, figsize=(16, 10), dpi=120)
    fig.suptitle('SPA Non-Newtonian: Linear vs Power-Law vs Adaptive MVS vs Full', fontsize=14)

    # Цвета: Linear=синий, Power-Law=оранж, Adaptive MVS=зелёный, Full=красный, Lookahead=фиолетовый
    colors = ['#2196F3', '#FF5722', '#4CAF50', '#D32F2F', '#9C27B0']
    styles = ['-', '-', '-', '-']

    for idx, (key, result) in enumerate(results.items()):
        c = colors[idx % len(colors)]

        # (0,0) Offset
        ax = axes[0, 0]
        ax.plot(result['t'], result['offset'], color=c, label=f'{titles[idx]}', lw=1.5)
        ax.set_ylabel('Offset [mm]')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)

        # (1,0) Ve
        ax = axes[1, 0]
        ax.plot(result['t'], result['ve_smooth'], color=c, label=titles[idx], lw=1.5)
        ax.set_ylabel('Ve [mm/s]')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)

        # (2,0) ΔP
        ax = axes[2, 0]
        ax.plot(result['t'], np.array(result['delta_p']) / 1e6, color=c, label=titles[idx], lw=1.5)
        ax.set_ylabel('ΔP [MPa]')
        ax.set_xlabel('Time [s]')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)

        # (0,1) Absolute Lag
        ax = axes[0, 1]
        lag = calc_lag(result)
        ax.plot(result['t'], lag['lag_abs'] * 1000, color=c,
                label=f'{titles[idx]} (max={lag["max_lag_abs"]*1000:.2f}μm)', lw=1.5)
        ax.set_ylabel('Lag [μm]')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)

        # (1,1) Relative Lag
        ax = axes[1, 1]
        ax.plot(result['t'], lag['lag_pct'], color=c,
                label=f'{titles[idx]} (max={lag["max_lag_pct"]:.1f}%)', lw=1.5)
        ax.set_ylabel('Lag [%]')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)

        # (2,1) Vf_max_eff
        ax = axes[2, 1]
        ax.plot(result['t'], result['vf_max_eff'], color=c, label=titles[idx], lw=1.5)
        ax.set_ylabel('Vf_max_eff [mm/s]')
        ax.set_xlabel('Time [s]')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=7)

    # (0,2) μ_app and γ̇ vs Ve
    ax = axes[0, 2]
    ax.set_title('μ_app and γ̇ vs Ve')
    cfg = NozzleConfig()
    nn = NonNewtonianModel(cfg)
    ve_r = np.linspace(0.01, 5, 100)
    q_r = ve_r * cfg.A_fil
    gamma_r = [nn.shear_rate_nozzle(q) for q in q_r]
    # Вязкость с диссипативным разогревом
    mu_r_cold = [nn.apparent_viscosity(g) for g in gamma_r]
    mu_r_hot = []
    for ve, q in zip(ve_r, q_r):
        dp = nn.pressure_drop(q)
        T = nn.calc_flow_temp(q, dp)
        g = nn.shear_rate_nozzle(q)
        mu_r_hot.append(nn.apparent_viscosity(g, T))

    ax2 = ax.twinx()
    ax.plot(ve_r, np.array(mu_r_cold) / 1000, 'b-', label='μ_app (изотерм.) [kPa·s]', lw=2)
    ax.plot(ve_r, np.array(mu_r_hot) / 1000, 'b--', label='μ_app (с разогревом) [kPa·s]', lw=2, alpha=0.7)
    ax2.plot(ve_r, gamma_r, 'r--', label='γ̇ [s⁻¹]', lw=1.5)
    ax.set_xlabel('Ve [mm/s]')
    ax.set_ylabel('μ_app [kPa·s]', color='b')
    ax2.set_ylabel('γ̇ [s⁻¹]', color='r')
    ax.grid(True, alpha=0.3)
    l1, la1 = ax.get_legend_handles_labels()
    l2, la2 = ax2.get_legend_handles_labels()
    ax.legend(l1 + l2, la1 + la2, loc='upper right', fontsize=7)

    # (1,2) Temp rise from dissipative heating
    ax = axes[1, 2]
    for idx, (key, result) in enumerate(results.items()):
        c = colors[idx % len(colors)]
        ax.plot(result['t'], result['temp_rise'], color=c, label=titles[idx], lw=1.5)
    ax.set_ylabel('ΔT_diss [K]')
    ax.set_xlabel('Time [s]')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=7)

    # (2,2) Adaptive MVS: Vf_max_eff vs Ve
    ax = axes[2, 2]
    ve_test = np.linspace(0, 5, 100)
    vf_base = 6.24
    alpha = cfg.alpha_mvs
    beta = cfg.beta_mvs
    ve_ref = cfg.ve_ref_mvs
    vf_adaptive = vf_base * (1 + alpha * (ve_test / ve_ref) ** beta)
    vf_adaptive = np.minimum(vf_adaptive, vf_base * 2)
    ax.plot(ve_test, vf_adaptive, 'g-', lw=2, label=f'α={alpha}, β={beta}')
    ax.axhline(y=vf_base, color='gray', linestyle='--', label=f'Vf_max_base={vf_base}mm/s')
    ax.set_xlabel('Ve [mm/s]')
    ax.set_ylabel('Vf_max_eff [mm/s]')
    ax.set_title('Adaptive MVS: Vf_max_eff(Ve)')
    ax.grid(True, alpha=0.3)
    ax.legend(fontsize=8)

    plt.tight_layout()
    plt.savefig(filename, dpi=120)
    plt.close()
    print(f"  График сохранён: {filename}")


def plot_block_comparison(results: Dict[str, dict], titles: List[str],
                          block_boundaries: List[int], dt: float,
                          filename: str):
    """Сравнение режимов для блочной симуляции с отметками стыков"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 8), dpi=120)
    fig.suptitle('LAPA Block-Based Simulation: SRL vs Pre-comp', fontsize=14)

    colors = ['#2196F3', '#FF5722', '#4CAF50', '#D32F2F', '#9C27B0']

    for idx, (key, result) in enumerate(results.items()):
        c = colors[idx % len(colors)]

        # (0,0) Offset
        ax = axes[0, 0]
        ax.plot(result['t'], result['offset'], color=c, label=titles[idx], lw=1.5)
        ax.plot(result['t'], result['offset_target'], color=c, linestyle=':', lw=1.0, alpha=0.5)
        for b in block_boundaries:
            ax.axvline(x=b * dt, color='gray', linestyle='--', alpha=0.3)
        ax.set_ylabel('Offset [mm]')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)

        # (0,1) Absolute Lag
        ax = axes[0, 1]
        lag = calc_lag(result)
        ax.plot(result['t'], lag['lag_abs'] * 1000, color=c,
                label=f'{titles[idx]} (max={lag["max_lag_abs"]*1000:.2f}um)', lw=1.5)
        for b in block_boundaries:
            ax.axvline(x=b * dt, color='gray', linestyle='--', alpha=0.3)
        ax.set_ylabel('Lag [um]')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)

        # (1,0) Ve
        ax = axes[1, 0]
        ax.plot(result['t'], result['ve_smooth'], color=c, label=titles[idx], lw=1.5)
        for b in block_boundaries:
            ax.axvline(x=b * dt, color='gray', linestyle='--', alpha=0.3)
        ax.set_ylabel('Ve [mm/s]')
        ax.set_xlabel('Time [s]')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)

        # (1,1) Integral Lag
        ax = axes[1, 1]
        lag_abs = lag['lag_abs']
        int_lag = np.cumsum(lag_abs) * dt
        ax.plot(result['t'], int_lag * 1000, color=c,
                label=f'{titles[idx]} (total={lag["integral_lag"]*1000:.2f}um*s)', lw=1.5)
        for b in block_boundaries:
            ax.axvline(x=b * dt, color='gray', linestyle='--', alpha=0.3)
        ax.set_ylabel('Integral Lag [um*s]')
        ax.set_xlabel('Time [s]')
        ax.grid(True, alpha=0.3)
        ax.legend(fontsize=8)

    plt.tight_layout()
    plt.savefig(filename, dpi=120)
    plt.close()
    print(f"  График сохранён: {filename}")


def print_results(results: Dict[str, dict], titles: List[str]):
    """Таблица метрик"""
    sep = f"\n{'='*100}"
    print(sep)
    print(f"{'Mode / Profile':<30} {'Max Lag [μm]':>15} {'Max Lag [%]':>15} {'Integral [μm·s]':>20} {'ΔT_diss [K]':>12} {'Vf_max [mm/s]':>14}")
    print(f"{'='*100}")
    for idx, (key, result) in enumerate(results.items()):
        lag = calc_lag(result)
        max_dt = float(np.max(result['temp_rise']))
        avg_vf = float(np.mean(result['vf_max_eff']))
        print(f"{titles[idx]:<30} {lag['max_lag_abs']*1000:>15.2f} {lag['max_lag_pct']:>14.1f}% {lag['integral_lag']*1000:>19.2f} {max_dt:>11.2f} {avg_vf:>13.2f}")
    print(f"{'='*100}")


# ═══════════════════════════════════════════════════════════════════════
# 6. Анализ K vs Ve (PA Tower)
# ═══════════════════════════════════════════════════════════════════════

def analyze_k_vs_ve():
    """Как меняется оптимальный K в зависимости от скорости (с разогревом)"""
    print(f"\n{'='*90}")
    print("АНАЛИЗ: Оптимальный K vs скорость печати (с диссипативным разогревом)")
    print(f"{'='*90}")

    cfg = NozzleConfig()
    nn = NonNewtonianModel(cfg)
    speeds = [7, 20, 30, 50, 80, 120, 150]
    lh, ew = 0.2, 0.44

    q_nom = 50 * lh * ew
    ve_nom = q_nom / cfg.A_fil
    k_nom = cfg.K_pa

    print(f"\n{'XY [mm/s]':>12} {'Ve [mm/s]':>10} {'γ̇ [s⁻¹]':>10} {'μ_app [kPa·s]':>16} {'T [°C]':>8} {'ΔP [MPa]':>10} {'K_req':>10}")
    print(f"{'-'*76}")

    results = []
    for v in speeds:
        Q = v * lh * ew
        Ve = Q / cfg.A_fil
        gamma = nn.shear_rate_nozzle(Q)
        dp = nn.pressure_drop(Q)
        T = nn.calc_flow_temp(Q, dp)
        mu = nn.apparent_viscosity(gamma, T) / 1000
        dp_mpa = nn.pressure_drop(Q, T) / 1e6
        t_c = T - 273.15

        # K_req для power-law: K_req = K_nom * (Ve/Ve_nom)^(n-1)
        # С температурной поправкой: дополнительный фактор exp[b*(1/T - 1/T_ref)]
        factor_temp = np.exp(nn.B_ACT * (1.0 / T - 1.0 / nn.T_REF))
        k_req = k_nom * (Ve / ve_nom) ** (cfg.n_power - 1) * factor_temp

        results.append((v, Ve, gamma, mu, t_c, dp_mpa, k_req))
        print(f"{v:>12} {Ve:>10.3f} {gamma:>10.0f} {mu:>15.3f} {t_c:>7.1f} {dp_mpa:>10.3f} {k_req:>10.4f}")

    print(f"{'-'*76}")
    print(f"  n = {cfg.n_power},  α_mvs = {cfg.alpha_mvs}")

    # График K vs Ve (сравнение: изотермический vs с разогревом)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5), dpi=100)

    xy_speeds = [r[0] for r in results]
    k_vals = [r[6] for r in results]

    ax1.plot(xy_speeds, k_vals, 'o-', color='#D32F2F', lw=2, markersize=8,
             label='K_req (с разогревом)')
    ax1.axhline(y=k_nom, color='#2196F3', linestyle='--', label=f'K_nom = {k_nom}')
    ax1.set_xlabel('XY Speed [mm/s]')
    ax1.set_ylabel('Optimal K')
    ax1.set_title('Required K vs Print Speed')
    ax1.grid(True, alpha=0.3)
    ax1.legend()

    # Температура
    temps = [r[4] for r in results]
    ax2.plot(xy_speeds, temps, 'o-', color='#FF5722', lw=2, markersize=8)
    ax2.axhline(y=240, color='gray', linestyle='--', label='T_set = 240°C')
    ax2.set_xlabel('XY Speed [mm/s]')
    ax2.set_ylabel('Melt Temperature [°C]')
    ax2.set_title('Dissipative Heating: T_melt(Ve)')
    ax2.grid(True, alpha=0.3)
    ax2.legend()

    plt.tight_layout()
    plt.savefig('spa_k_vs_ve.png', dpi=100)
    plt.close()
    print("  График сохранён: spa_k_vs_ve.png")


# ═══════════════════════════════════════════════════════════════════════
# 7. Анализ Adaptive MVS: влияние на SRL limit
# ═══════════════════════════════════════════════════════════════════════

def analyze_adaptive_mvs():
    """Влияние Adaptive MVS на SRL limit при разных Ve"""
    print(f"\n{'='*90}")
    print("АНАЛИЗ: Adaptive MVS — влияние на SRL ceiling")
    print(f"{'='*90}")

    cfg = NozzleConfig()
    ftm_fs = 5000
    srl_div = 2

    print(f"\n{'Ve [mm/s]':>12} {'Vf_max_base':>14} {'Vf_max_eff':>14} {'c_limit_base':>14} {'c_limit_eff':>14} {'mvs_limit_base':>16} {'mvs_limit_eff':>16}")
    print(f"{'-'*100}")

    for ve in [0.1, 0.256, 0.5, 1.0, 1.829, 3.0, 5.0]:
        vf_base = 6.24
        vf_eff = vf_base * (1 + cfg.alpha_mvs * (ve / cfg.ve_ref_mvs) ** cfg.beta_mvs)
        vf_eff = min(vf_eff, vf_base * 2)

        c_limit_base = vf_base / (ftm_fs * srl_div)
        c_limit_eff = vf_eff / (ftm_fs * srl_div)

        avail_base = vf_base - abs(ve)
        avail_eff = vf_eff - abs(ve)
        mvs_base = max(avail_base, vf_base) / ftm_fs
        mvs_eff = max(avail_eff, vf_eff) / ftm_fs

        print(f"{ve:>12.3f} {vf_base:>14.2f} {vf_eff:>14.2f} "
              f"{c_limit_base*1000:>14.4f} {c_limit_eff*1000:>14.4f} "
              f"{mvs_base*1000:>15.4f} {mvs_eff*1000:>15.4f}")

    print(f"\n  Вывод: Adaptive MVS увеличивает c_limit на high Ve,")
    print(f"  позволяя offset быстрее нарастать без превышения MVS ceiling.")


# ═══════════════════════════════════════════════════════════════════════
# 8. MAIN
# ═══════════════════════════════════════════════════════════════════════

def main():
    import sys

    print("=" * 90)
    print("  SPA Non-Newtonian FDM Simulator v2.0")
    print("  Моделирование неньютоновского течения в сопле FDM-принтера")
    print("=" * 90)

    cfg = NozzleConfig()
    nn = NonNewtonianModel(cfg)

    # Базовые параметры
    print(f"\n  Сопло: {cfg.D_nozzle}mm × {cfg.L_nozzle}mm")
    print(f"  Филамент: {cfg.D_filament}mm, n={cfg.n_power}, K_cons={cfg.K_consistency} Pa·s^n")
    print(f"  K_PA = {cfg.K_pa} (при XY=50mm/s, Ve={cfg.ve_ref_mvs:.3f}mm/s)")
    print(f"  Adaptive MVS: α={cfg.alpha_mvs}, β={cfg.beta_mvs}")

    # Скорость сдвига при разных режимах
    print(f"\n  Скорость сдвига в сопле (с диссипативным разогревом):")
    for v_xy, label in [(7, 'notch (7mm/s)'), (30, 'slow (30mm/s)'),
                        (50, 'normal (50mm/s)'), (100, 'fast (100mm/s)')]:
        Q = v_xy * 0.2 * 0.44
        Ve = Q / cfg.A_fil
        gamma = nn.shear_rate_nozzle(Q)
        dp = nn.pressure_drop(Q)
        T = nn.calc_flow_temp(Q, dp)
        mu = nn.apparent_viscosity(gamma, T)
        t_c = T - 273.15
        print(f"    {label:>20}: Ve={Ve:.3f}mm/s, T={t_c:.1f}°C, "
              f"γ̇={gamma:.0f}s⁻¹, μ_app={mu:.0f}Pa·s")

    # Профили
    # Тестовые детали: 50mm/s baseline
    profiles = [
        MotionProfile("90° Wall (50mm/s)", 50, 35, 90),
        MotionProfile("Notch Entry (50mm/s)", 50, 7, 90, 50),
        MotionProfile("Notch Exit (50mm/s)", 7, 50, 90, 7),
    ]
    # Ваш профиль: 200mm/s, JD=0.03 и JD=0.06
    v_fast = 200  # mm/s
    accel = 3000  # mm/s²
    for jd_val in [0.03, 0.06]:
        p = MotionProfile(f"Fast 90° Wall (JD={jd_val})", v_fast, 35, 90, jd=jd_val)
        profiles.append(p)
        p = MotionProfile(f"Fast Notch Entry (JD={jd_val})", v_fast, 7, 90, v_fast, jd=jd_val)
        profiles.append(p)
        p = MotionProfile(f"Fast Notch Exit (JD={jd_val})", 7, v_fast, 90, 7, jd=jd_val)
        profiles.append(p)

    # Предрасчёт Ve
    for p in profiles:
        p.ve_before, p.ve_after = p.calc_ve(cfg)
        if p.jd > 0:
            vj = p.calc_junction_speed(accel)
            # Время декелерации: (v_before - v_j) / accel [s]
            p.decel_time = (p.v_xy_before - vj) / accel
            # Длительность фазы изменения Ve в кадрах
            p.decel_frames = int(p.decel_time * 5000)
        else:
            p.decel_time = 0
            p.decel_frames = 20  # sigmoid transition ~20 frames

    # Режимы для сравнения
    modes = [
        ('linear',       'Linear PA (v4.11)'),
        ('power_law',    'Power-Law PA'),
        ('adaptive_mvs', 'Adaptive MVS'),
        ('full',         'Power-Law + MVS'),
        ('lookahead',    'PA Lookahead'),
        ('precomp',      'Pre-comp (Klipper-like)'),
    ]

    # Пересчитываем lookahead: для каждого профиля отдельно
    # Размер lookahead = число кадров, нужное SRL для полного изменения offset
    # Δoffset_target = K × ΔVe
    # SRL rate = Vf_max / (FS × DIVIDER) = 6.24 / (5000 × 2) = 0.000624 mm/frame
    # frames_needed = Δoffset_target / SRL_rate
    SRL_RATE = 6.24 / (5000 * 2)  # 0.000624 mm/frame

    # Вычисляем lookahead для самого тяжёлого профиля
    max_la_frames = 0
    for p in profiles:
        dVe = abs(p.ve_before - p.ve_after)
        dOff = cfg.K_pa * dVe
        frames_needed = int(dOff / SRL_RATE + 0.5)
        if frames_needed > max_la_frames:
            max_la_frames = frames_needed

    # Ограничиваем: lookahead не дольше времени декелерации + запас
    # Для 200mm/s, JD=0.03: decel = 63ms = 317 frames
    # lookahead не должен быть короче decel_frames
    la_frames = max(max_la_frames, max(p.decel_frames for p in profiles) + 10)
    # Ограничение: не более 30% от всех кадров симуляции
    la_frames = min(la_frames, int(0.3 * 5000 * 0.3))  # 450 frames max

    sim = SPA_Simulator(cfg, lookahead_frames=la_frames)

    print(f"\n  SRL rate: {SRL_RATE*1000:.4f} μm/frame")
    print(f"  Lookahead: {la_frames} frames = {la_frames/5000*1000:.1f}ms (из расчёта самого тяжёлого профиля)")
    for p in profiles:
        dVe = abs(p.ve_before - p.ve_after)
        dOff = cfg.K_pa * dVe
        frames_needed = int(dOff / SRL_RATE + 0.5)
        print(f"    {p.name:<30}: ΔVe={dVe:.3f}mm/s, Δoffset={dOff*1000:.1f}μm, "
              f"SRL needs {frames_needed:>4} frames = {frames_needed/5000*1000:.1f}ms"
              + (f", decel={p.decel_frames}fr" if hasattr(p, 'decel_frames') else ""))
    all_results = {}
    all_titles = []

    # Для каждого профиля — все 4 режима
    for p in profiles:
        print(f"\n{'─'*70}")
        print(f"  Profile: {p.name} — Ve: {p.ve_before:.3f} → {p.ve_after:.3f} mm/s")
        print(f"{'─'*70}")

        for mode_key, mode_name in modes:
            r = sim.simulate(p, duration=0.3, mode=mode_key)
            key = f"{p.name}_{mode_key}"
            title = f"{p.name} ({mode_name})"
            all_results[key] = r
            all_titles.append(title)

            ll = calc_lag(r)
            print(f"    {mode_name:<20}: lag={ll['max_lag_abs']*1000:>8.2f}μm "
                  f"({ll['max_lag_pct']:>6.1f}%), "
                  f"Vf_eff={np.mean(r['vf_max_eff']):.2f}mm/s, "
                  f"ΔT={np.max(r['temp_rise']):.2f}K")

    # Таблица всех результатов
    print_results(all_results, all_titles)

    # График для каждого профиля отдельно
    for p in profiles:
        sub_results = {}
        sub_titles = []
        for mode_key, mode_name in modes:
            key = f"{p.name}_{mode_key}"
            sub_results[key] = all_results[key]
            sub_titles.append(mode_name)
        safe_name = p.name.lower().replace(" ", "_").replace("°", "deg").replace("(", "").replace(")", "").replace("=", "_").replace("/", "_")
        plot_comparison(sub_results, sub_titles,
                        f'spa_{safe_name}.png')

    # Анализ K vs Ve
    analyze_k_vs_ve()

    # ═══════════════════════════════════════════════════════════════════
    # LAPA Block-Based Simulation
    # ═══════════════════════════════════════════════════════════════════
    print(f"\n{'='*90}")
    print("  LAPA BLOCK-BASED SIMULATION")
    print(f"{'='*90}")

    # Базовые параметры для расчёта Ve
    lh, ew = 0.2, 0.44

    # Блочные профили
    # Угол 90°: после JD junction speed = 35mm/s (JD ≈ большой) или 9.5mm/s (JD=0.03)
    # Формула Ve: Q = v_xy * lh * ew, Ve = Q / A_fil
    A_fil = cfg.A_fil
    def calc_ve_from_speed(v_xy):
        return v_xy * lh * ew / A_fil

    ve_50  = calc_ve_from_speed(50)    # 1.829
    ve_35  = calc_ve_from_speed(35)    # 1.281
    ve_200 = calc_ve_from_speed(200)   # 7.317
    ve_jd  = calc_ve_from_speed(9.5)   # 0.347 (JD=0.03)
    ve_7   = calc_ve_from_speed(7)     # 0.256
    ve_commute = calc_ve_from_speed(50) # 1.829 (после accel)

    block_profiles = [
        BlockSequence("90deg Wall 50mm/s", [
            Block("approach", v_xy=50, ve=ve_50, length_mm=5.0),
            Block("corner",   v_xy=35, ve=ve_35, length_mm=2.0),
            Block("depart",   v_xy=50, ve=ve_50, length_mm=5.0),
        ]),
        BlockSequence("Fast Wall JD=0.03", [
            Block("approach", v_xy=200, ve=ve_200, length_mm=20.0),
            Block("decel", v_xy=200, ve=0, length_mm=6.65, accel=3000,
                  ve_start=ve_200, ve_end=ve_jd),
            Block("corner", v_xy=9.5, ve=ve_jd, length_mm=0.5),
            Block("accel", v_xy=50, ve=0, length_mm=6.65, accel=3000,
                  ve_start=ve_jd, ve_end=ve_commute),
            Block("depart", v_xy=50, ve=ve_commute, length_mm=20.0),
        ]),
        BlockSequence("Notch Entry 200mm/s", [
            Block("approach", v_xy=200, ve=ve_200, length_mm=20.0),
            Block("decel", v_xy=200, ve=0, length_mm=6.65, accel=3000,
                  ve_start=ve_200, ve_end=ve_jd),
            Block("notch", v_xy=7, ve=ve_7, length_mm=2.0),
        ]),
        BlockSequence("Notch Exit 200mm/s", [
            Block("notch", v_xy=7, ve=ve_7, length_mm=2.0),
            Block("accel", v_xy=50, ve=0, length_mm=6.65, accel=3000,
                  ve_start=ve_7, ve_end=ve_commute),
            Block("depart", v_xy=50, ve=ve_commute, length_mm=20.0),
        ]),
    ]

    block_modes = [
        ('linear',  'SRL (v4.11)'),
        ('precomp', 'Pre-comp (LAPA v1.0)'),
        ('adaptive_lookahead', 'Adaptive (LAPA v1.1)'),
    ]

    print(f"\n{'='*130}")
    print(f"{'Block Sequence':<30} {'Mode':<25} {'Max Lag [um]':>15} {'Max Lag [%]':>15} {'Integral [um*s]':>20}")
    print(f"{'='*130}")

    for seq in block_profiles:
        print(f"\n  {seq.name}:")
        for mode_key, mode_name in block_modes:
            r = simulate_blocks(seq, cfg, mode=mode_key, lookahead_frames=la_frames)
            ll = calc_lag(r)
            print(f"    {mode_name:<25}: lag={ll['max_lag_abs']*1000:>8.2f}um "
                  f"({ll['max_lag_pct']:>6.1f}%), "
                  f"int_lag={ll['integral_lag']*1000:>8.2f}um*s")

        # График для каждого сценария
        sub_results = {}
        sub_titles = []
        block_boundaries = []
        for mode_key, mode_name in block_modes:
            r = simulate_blocks(seq, cfg, mode=mode_key, lookahead_frames=la_frames)
            sub_results[mode_key] = r
            sub_titles.append(mode_name)
            if not block_boundaries:
                block_boundaries = r.get('block_boundaries', [])

        safe_name = seq.name.lower().replace(" ", "_").replace("°", "deg").replace("(", "").replace(")", "").replace("=", "_").replace("/", "_")
        plot_block_comparison(sub_results, sub_titles, block_boundaries, 1.0/5000.0,
                              f'lapa_{safe_name}.png')

    print(f"\n{'='*90}")
    print("  LAPA v1.1 SIMULATION COMPLETE")
    print(f"{'='*90}")


if __name__ == '__main__':
    main()
