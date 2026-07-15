/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2025 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
#include "../../../inc/MarlinConfigPre.h"

#if ENABLED(FT_MOTION)

#include "../../gcode.h"
#include "../../../module/ft_motion.h"
#include "../../../module/stepper.h"
#include "../../../module/planner.h"
#include "../../../lcd/marlinui.h"

void say_ftm_settings() {
  #if ENABLED(FTM_POLYS)
    const ft_config_t &c = ftMotion.cfg;
  #endif

  #if HAS_FTM_TRAJECTORY_SELECTION
    SERIAL_ECHOLN(F("  Trajectory: "), ftMotion.getTrajectoryName(), C('('), (uint8_t)ftMotion.getTrajectoryType(), C(')'));
    #if ENABLED(FTM_POLYS)
      if (ftMotion.getTrajectoryType() == TrajectoryType::POLY6)
        SERIAL_ECHOLNPGM("  Poly6 Overshoot: ", p_float_t(c.poly6_acceleration_overshoot, 3));
    #endif
  #endif
}

void GcodeSuite::M494_report(const bool forReplay/*=true*/) {
  TERN_(MARLIN_SMALL_BUILD, return);

  report_heading_etc(forReplay, F("FT Motion"));
  SERIAL_ECHOPGM("  M494 T", (uint8_t)ftMotion.getTrajectoryType());

  #if ENABLED(FTM_POLYS)
    const ft_config_t &c = ftMotion.cfg;
    if (ftMotion.getTrajectoryType() == TrajectoryType::POLY6)
      SERIAL_ECHOPGM(" O", c.poly6_acceleration_overshoot);
  #endif

  SERIAL_EOL();
}

/**
 * M494: Set Fixed-time Motion Control parameters
 *
 * Parameters:
 *    T<type> Set trajectory generator type (0=TRAPEZOIDAL, 1=POLY5, 2=POLY6)
 *    O<overshoot> Set acceleration overshoot for POLY6 (1.25-1.875)
 *    X<time> Set smoothing time for the X axis
 *    Y<time> Set smoothing time for the Y axis
 *    Z<time> Set smoothing time for the Z axis
 *    E<time> Set smoothing time for the E axis
 */
void GcodeSuite::M494() {
  bool report = !parser.seen_any();

  #if HAS_FTM_TRAJECTORY_SELECTION

    // Parse trajectory type parameter.
    if (parser.seenval('T')) {
      if (ftMotion.updateTrajectoryType(TrajectoryType(parser.value_int())))
        report = true;
      else
        SERIAL_ECHOLN(F("?Invalid (T)rajectory type value. (0=TRAPEZOIDAL"
          TERN_(FTM_POLYS, ", 1=POLY5, 2=POLY6")
          ")"));
    }

  #endif // HAS_FTM_TRAJECTORY_SELECTION

  #if ENABLED(FTM_POLYS)

    // Parse overshoot parameter.
    if (parser.seenval('O')) {
      const float val = parser.value_float();
      if (WITHIN(val, 1.25f, 1.875f)) {
        ftMotion.cfg.poly6_acceleration_overshoot = val;
        report = true;
      }
      else
        SERIAL_ECHOLN(F("?Invalid "), F("(O)vershoot value. Range 1.25-1.875"));
    }

  #endif // FTM_POLYS

  if (report) {
    ui.refresh();
    say_ftm_settings();
  }
}

#endif // FT_MOTION
