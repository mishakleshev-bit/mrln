/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2023 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

#include "../inc/MarlinConfigPre.h" // Access the top level configurations.
#include "planner.h"      // Access block type from planner.
#include "stepper.h"      // For stepper motion and direction

#include "ft_motion/trajectory_trapezoidal.h"
#if ENABLED(FTM_POLYS)
  #include "ft_motion/trajectory_poly5.h"
  #include "ft_motion/trajectory_poly6.h"
#endif
#if ENABLED(RESONANCE_TEST)
  #include "../feature/resonance/resonance_generator.h"
#endif

#if HAS_FTM_SHAPING
  #include "ft_motion/shaping.h"
#endif
#include "ft_motion/stepping.h"

#define FTM_VERSION   2   // Change version when hosts need to know

// Maximum LA correction per frame (mm) to prevent stepper rate overflow.
// Can be overridden in Configuration_adv.h before including this file.
#ifndef FT_MOTION_LA_CLAMP
  #define FT_MOTION_LA_CLAMP 10.0f
#endif

#if ENABLED(FTM_DYNAMIC_FREQ)
  #define HAS_DYNAMIC_FREQ 1
  #if HAS_Z_AXIS
    #define HAS_DYNAMIC_FREQ_MM 1
  #endif
  #if HAS_EXTRUDERS
    #define HAS_DYNAMIC_FREQ_G 1
  #endif
#endif

/**
 * FTConfig - The active configured state of FT Motion
 */
typedef struct FTConfig {
  #if HAS_STANDARD_MOTION
    bool active = ENABLED(FTM_IS_DEFAULT_MOTION);   // Active (else Standard Motion)
  #else
    static constexpr bool active = true;                  // Always active with NO_STANDARD_MOTION
  #endif
  bool axis_sync_enabled = true;                          // Axis synchronization enabled

  #if HAS_FTM_SHAPING
    ft_shaped_shaper_t shaper =                           // Shaper type
      SHAPED_ARRAY(FTM_DEFAULT_SHAPER_X, FTM_DEFAULT_SHAPER_Y, FTM_DEFAULT_SHAPER_Z, FTM_DEFAULT_SHAPER_E);
    ft_shaped_float_t baseFreq =                          // Base frequency. [Hz]
      SHAPED_ARRAY(FTM_SHAPING_DEFAULT_FREQ_X, FTM_SHAPING_DEFAULT_FREQ_Y, FTM_SHAPING_DEFAULT_FREQ_Z, FTM_SHAPING_DEFAULT_FREQ_E);
    ft_shaped_float_t zeta =                              // Damping factor
      SHAPED_ARRAY(FTM_SHAPING_ZETA_X, FTM_SHAPING_ZETA_Y, FTM_SHAPING_ZETA_Z, FTM_SHAPING_ZETA_E);

    #if HAS_FTM_EI_SHAPING
      ft_shaped_float_t vtol =                              // Vibration Level
        SHAPED_ARRAY(FTM_SHAPING_V_TOL_X, FTM_SHAPING_V_TOL_Y, FTM_SHAPING_V_TOL_Z, FTM_SHAPING_V_TOL_E);
    #endif

    #if HAS_DYNAMIC_FREQ
      dynFreqMode_t dynFreqMode = FTM_DEFAULT_DYNFREQ_MODE; // Dynamic frequency mode configuration.
      ft_shaped_float_t dynFreqK = { 0.0f };                // Scaling / gain for dynamic frequency. [Hz/mm] or [Hz/g]
    #else
      static constexpr dynFreqMode_t dynFreqMode = dynFreqMode_DISABLED;
    #endif

  #endif // HAS_FTM_SHAPING

  #if ENABLED(FTM_POLYS)
    float poly6_acceleration_overshoot; // Overshoot factor for Poly6 (1.25 to 2.0)
  #endif
  #if HAS_FTM_TRAJECTORY_SELECTION
    TrajectoryType trajectory_type = TrajectoryType::FTM_TRAJECTORY_TYPE; // Trajectory generator type
  #else
    static constexpr TrajectoryType trajectory_type = TrajectoryType::TRAPEZOIDAL;
  #endif
  static void prep_for_shaper_change();
  static void update_shaping_params();

  #if HAS_STANDARD_MOTION
    bool setActive(const bool a) {
      if (a == active) return false;
      stepper.ftMotion_syncPosition();
      prep_for_shaper_change();
      active = a;
      update_shaping_params();
      return true;
    }
  #endif

  bool setAxisSync(const bool ena) {
    if (ena == axis_sync_enabled) return false;
    prep_for_shaper_change();
    axis_sync_enabled = ena;
    update_shaping_params();
    return true;
  }

  #if HAS_FTM_SHAPING

    bool setShaper(const AxisEnum a, const ftMotionShaper_t s) {
      if (s == shaper[a]) return false;
      prep_for_shaper_change();
      shaper[a] = s;
      update_shaping_params();
      return true;
    }

    constexpr bool goodZeta(const float z) { return WITHIN(z, 0.00f, ftm_max_dampening); }

    bool setZeta(const AxisEnum a, float z) {
      if (z == zeta[a]) return false;
      LIMIT(z, 0.00f, ftm_max_dampening);
      prep_for_shaper_change();
      zeta[a] = z;
      update_shaping_params();
      return true;
    }

    #if HAS_FTM_EI_SHAPING

      constexpr bool goodVtol(const float v) { return WITHIN(v, 0.00f, 1.0f); }

      bool setVtol(const AxisEnum a, float v) {
        if (v == vtol[a]) return false;
        LIMIT(v, 0.00f, 1.0f);
        prep_for_shaper_change();
        vtol[a] = v;
        update_shaping_params();
        return true;
      }

    #endif

    #if HAS_DYNAMIC_FREQ

      uint8_t setDynFreqMode(const uint8_t m) {
        if (dynFreqMode_t(m) == dynFreqMode) return 0;
        switch (dynFreqMode_t(m)) {
          default: return 2;
          TERN_(HAS_DYNAMIC_FREQ_MM, case dynFreqMode_Z_BASED:)
          TERN_(HAS_DYNAMIC_FREQ_G, case dynFreqMode_MASS_BASED:)
          case dynFreqMode_DISABLED:
            prep_for_shaper_change();
            dynFreqMode = dynFreqMode_t(m);
            break;
        }
        update_shaping_params();
        return 1;
      }

      bool modeUsesDynFreq() const {
        return (TERN0(HAS_DYNAMIC_FREQ_MM, dynFreqMode == dynFreqMode_Z_BASED)
             || TERN0(HAS_DYNAMIC_FREQ_G,  dynFreqMode == dynFreqMode_MASS_BASED));
      }

      bool setDynFreqK(const AxisEnum a, const float k) {
        if (!modeUsesDynFreq()) return false;
        if (k == dynFreqK[a]) return false;
        prep_for_shaper_change();
        dynFreqK[a] = k;
        update_shaping_params();
        return true;
      }

    #endif // HAS_DYNAMIC_FREQ

  #endif // HAS_FTM_SHAPING

  constexpr bool goodBaseFreq(const float f) { return WITHIN(f, FTM_MIN_SHAPE_FREQ, (FTM_FS) / 2); }

  bool setBaseFreq(const AxisEnum a, float f) {
    if (f == baseFreq[a]) return false;
    LIMIT(f, FTM_MIN_SHAPE_FREQ, (FTM_FS) / 2);
    prep_for_shaper_change();
    baseFreq[a] = f;
    update_shaping_params();
    return true;
  }

  void set_defaults() {
    #if HAS_STANDARD_MOTION
      active = ENABLED(FTM_IS_DEFAULT_MOTION);
    #endif

    #if HAS_FTM_SHAPING

      #define _SET_CFG_DEFAULTS(A) do{ \
        shaper.A   = FTM_DEFAULT_SHAPER_##A; \
        baseFreq.A = FTM_SHAPING_DEFAULT_FREQ_##A; \
        zeta.A     = FTM_SHAPING_ZETA_##A; \
        TERN_(HAS_FTM_EI_SHAPING, vtol.A = FTM_SHAPING_V_TOL_##A); \
      }while(0);

      SHAPED_MAP(_SET_CFG_DEFAULTS);
      #undef _SET_CFG_DEFAULTS

      #if HAS_DYNAMIC_FREQ
        dynFreqMode = FTM_DEFAULT_DYNFREQ_MODE;
        dynFreqK.reset();
      #endif

    #endif // HAS_FTM_SHAPING

    TERN_(FTM_POLYS, poly6_acceleration_overshoot = FTM_POLY6_ACCELERATION_OVERSHOOT);

    update_shaping_params();
  }

} ft_config_t;

/**
 * FTMotion - Singleton class encapsulating Fixed Time Motion
 */
class FTMotion {

  public:

    // Public variables
    static ft_config_t cfg;
    static bool busy;

    static void set_defaults() {
      cfg.set_defaults();

      #if HAS_FTM_TRAJECTORY_SELECTION
        setTrajectoryType(TrajectoryType::FTM_TRAJECTORY_TYPE);
      #endif

      reset();
    }

    static AxisBits moving_axis_flags,                    // These axes are moving in the planner block being processed
                    axis_move_dir;                        // ...in these directions

    // Public methods
    static void init();
    static void loop();                                   // Controller main, to be invoked from non-isr task.

    static void reset();                                  // Reset all states of the fixed time conversion to defaults.

    // Safely toggle the active state of FT Motion
    #if ALL(FT_MOTION, HAS_STANDARD_MOTION)
      static bool toggle() {
        stepper.ftMotion_syncPosition();
        cfg.setActive(!cfg.active);
        update_shaping_params();
        return cfg.active;
      }
    #endif

    // Trajectory generator selection
    #if HAS_FTM_TRAJECTORY_SELECTION
      static void setTrajectoryType(const TrajectoryType type);
      static bool updateTrajectoryType(const TrajectoryType type);
    #endif
    static TrajectoryType getTrajectoryType() { return TERN(HAS_FTM_TRAJECTORY_SELECTION, trajectoryType, TrajectoryType::TRAPEZOIDAL); }
    static FSTR_P getTrajectoryName();

    FORCE_INLINE static bool axis_is_moving(const AxisEnum real) {
      return cfg.active ? moving_axis_flags[real] : TERN0(HAS_STANDARD_MOTION, stepper.axis_is_moving(real));
    }
    FORCE_INLINE static bool axis_direction(const AxisEnum real) {
      return cfg.active ? axis_move_dir[real] : stepper.last_direction_bits[real];
    }

    // A frame of the stepping plan
    static stepping_t stepping;

    // Add a single set of coordinates in the stepping plan
    FORCE_INLINE static void stepping_enqueue(const xyze_float_t traj_coords) {
      #define _TOSTEPS_q16(A, B) int64_t(traj_coords.A * planner.settings.axis_steps_per_mm[B] * (1ULL << 16))
      XYZEval<int64_t> next_steps_q48_16 = LOGICAL_AXIS_ARRAY(
        _TOSTEPS_q16(e, block_extruder_axis),
        _TOSTEPS_q16(x, X_AXIS), _TOSTEPS_q16(y, Y_AXIS), _TOSTEPS_q16(z, Z_AXIS),
        _TOSTEPS_q16(i, I_AXIS), _TOSTEPS_q16(j, J_AXIS), _TOSTEPS_q16(k, K_AXIS),
        _TOSTEPS_q16(u, U_AXIS), _TOSTEPS_q16(v, V_AXIS), _TOSTEPS_q16(w, W_AXIS)
      );
      #undef _TOSTEPS_q16
      stepping.enqueue(next_steps_q48_16);
    }

    #if HAS_FTM_DIR_CHANGE_HOLD
      static xyze_float_t ftm_hold_frames(xyze_float_t hold_coords);
      #if ENABLED(RESONANCE_TEST)
        xyze_float_t get_last_target_traj() { return last_target_traj; } ;
      #endif
    #endif

  private:
    // Block data variables.
    static xyze_pos_t   startPos,         // (mm) Start position of block
                        endPos_prevBlock, // (mm) End position of previous block
                        last_target_traj; // (mm) Last target position after shaping and smoothing
    static xyze_float_t ratio;            // (ratio) Axis move ratio of block
    static float tau;                     // (s) Time since start of block
    static bool fastForwardUntilMotion;   // Fast forward time if there is no motion

    #if HAS_FTM_DIR_CHANGE_HOLD
      static xyze_uint_t hold_frames;     // Briefly hold motion after direction changes to fix TMC2208 bug
      static AxisBits last_traj_dir;      // Direction of the last trajectory point after shaping, smoothing, ...
    #endif

    // Trajectory generators
    static TrapezoidalTrajectoryGenerator trapezoidalGenerator;
    #if ENABLED(FTM_POLYS)
      static Poly5TrajectoryGenerator poly5Generator;
      static Poly6TrajectoryGenerator poly6Generator;
    #endif
    #if HAS_FTM_TRAJECTORY_SELECTION
      static TrajectoryType trajectoryType;
      static TrajectoryGenerator* currentGenerator;
    #else
      static constexpr TrajectoryGenerator *currentGenerator = &trapezoidalGenerator;
    #endif

    #if FTM_HAS_LIN_ADVANCE
      static bool use_advance_lead;
    #endif

    #if ENABLED(DISTINCT_E_FACTORS)
      static uint8_t block_extruder_axis;  // Cached extruder axis index
    #elif HAS_EXTRUDERS
      static constexpr uint8_t block_extruder_axis = E_AXIS;
    #endif

    #if HAS_FTM_SHAPING
      static shaping_t shaping; // Shaping data
    #endif

    // Dual-pipeline: E axis delay buffer for phase synchronization
    // When shaping is enabled on XYZ but not E, the shaped XYZ outputs
    // lag behind the unshaped E output by the shaping centroid delay.
    // E gets delayed by largest_delay_samples frames to stay in sync.
    #if defined(HAS_EXTRUDERS) && defined(HAS_FTM_SHAPING)
      static float e_delay_buffer[ftm_zmax + 1];
      static uint32_t e_delay_wr_idx;
      // Enqueue current E, return delayed E from largest_delay_samples frames ago
      static float e_delay_enqueue(const float e_coord) {
        e_delay_buffer[e_delay_wr_idx] = e_coord;
        // Dynamic delay = largest shaping delay for phase sync
        const uint32_t D = shaping.largest_delay_samples;
        int32_t rd = int32_t(e_delay_wr_idx) - int32_t(D);
        if (rd < 0) rd += ftm_zmax + 1;
        const float delayed_e = e_delay_buffer[rd];
        if (++e_delay_wr_idx > ftm_zmax) e_delay_wr_idx = 0;
        return delayed_e;
      }
      static void e_delay_fill(const float val) {
        for (uint32_t i = 0; i <= ftm_zmax; ++i) e_delay_buffer[i] = val;
      }
    #endif

    #if HAS_FTM_SHAPING
      // Refresh gains and indices used by shaping functions.
      friend void ft_config_t::update_shaping_params();
      static void update_shaping_params();
    #endif

    // Synchronize and reset motion prior to parameter changes
    friend void ft_config_t::prep_for_shaper_change();
    static void prep_for_shaper_change() {
      // planner.synchronize guarantees that motion reached a standstill with no echoes pending execution (including a runout block)
      planner.synchronize();
      // Set the next starting position to the exact reached position.
      endPos_prevBlock = last_target_traj;
      // We now know that we are not moving and there are no pending echoes,
      // so set all shaping buffers to current position in case the new shaping
      // parameters force input shaping to look in a past position for echoes.
      shaping.fill(endPos_prevBlock);
      // Reset E delay buffer to current position
      #if defined(HAS_EXTRUDERS) && defined(HAS_FTM_SHAPING)
        e_delay_fill(endPos_prevBlock.e);
      #endif
      fastForwardUntilMotion = true;
    }

    // Buffers
    static void discard_planner_block_protected();
    static uint32_t calc_runout_samples();
    static void plan_runout_block();
    static void fill_stepper_plan_buffer();
    static xyze_float_t calc_traj_point(const float dist);
    static bool plan_next_block();
    static void ensure_extruder_float_precision() IF_DISABLED(HAS_EXTRUDERS, {});

}; // class FTMotion

extern FTMotion ftMotion; // Use ftMotion.thing, not FTMotion::thing.

/**
 * Optional behavior to turn FT Motion off for homing/probing.
 * Applies when FTM_HOME_AND_PROBE is disabled.
 */
#if DISABLED(FTM_HOME_AND_PROBE)
  typedef struct FTMotionDisableInScope {
    bool isactive;
    FTMotionDisableInScope() {
      isactive = ftMotion.cfg.active;
      ftMotion.cfg.setActive(false);
    }
    ~FTMotionDisableInScope() {
      ftMotion.cfg.setActive(isactive);
      if (isactive) ftMotion.init();
    }
  } FTMotionDisableInScope_t;
#endif

#define FTM_DISABLE_IN_SCOPE() TERN(FTM_HOME_AND_PROBE, NOOP, FTMotionDisableInScope FT_Disabler)
