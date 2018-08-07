#include "../../..//Marlin.h"
#include "../../api/api.h"
#include "../../api/diagnostics/diagnostics.h"

#include "../../../planner.h"

#include "../../vone/tools/NullTool.h"
#include "../../vone/tools/Probe.h"

static void s_start() {
  overrideLeds(0, 255, 0, 0); // blink green
  log << F("Starting manufacturing procedure") << endl;
}

static int s_end(tools::Tool& tool, int result) {
  if (result == 0) {
    const float bedCenterX = min_pos[X_AXIS] + (max_pos[X_AXIS] - min_pos[X_AXIS]) / 2;

    // Success, pulse leds and go to finished position
    // (trying to make it obvious that we are done)
    log << F("Manufacturing procedure completed successfully") << endl;
    raise();
    moveXY(tool, bedCenterX, max_pos[Y_AXIS]);

    overrideLeds(255, 255, 255, 3); // pulse white (which also helps confirm LEDs have all 3 colors)
  } else {
    logError << F("Manufacturing procedure failed") << endl;
    overrideLeds(255, 80, 0, 0); // blink yellow
  }
  return result;
}

int runBurnInSequence(tools::NullTool& noTool) {
  s_start();

  // Override speed settings
  // NOTE: using acceleration hides a problem in the y-axis mechanical system on some units
  //       so when we run burn-in we use the settings we had before enabling acceleration.
  log << F("Overriding feedrate and acceleration settings for burn-in") << endl;
  const auto maxFX = max_feedrate[ X_AXIS ];
  const auto maxFY = max_feedrate[ Y_AXIS ];
  const auto maxAX = max_acceleration_units_per_sq_second[ X_AXIS ];
  const auto maxAY = max_acceleration_units_per_sq_second[ Y_AXIS ];
  const auto maxA = acceleration;
  max_feedrate[ X_AXIS ] = 7500.0/60.0;
  max_feedrate[ Y_AXIS ] = 7500.0/60.0;
  max_acceleration_units_per_sq_second[ X_AXIS ] = 700;
  max_acceleration_units_per_sq_second[ Y_AXIS ] = 700;
  acceleration = 1000;
  reset_acceleration_rates();

  // Run burn-in
  auto result = s_end(noTool, burnInSequence(noTool));

  // Restore speed settings
  log << F("Restoring feedrate and acceleration settings") << endl;
  max_feedrate[ X_AXIS ] = maxFX;
  max_feedrate[ Y_AXIS ] = maxFY;
  max_acceleration_units_per_sq_second[ X_AXIS ] = maxAX;
  max_acceleration_units_per_sq_second[ Y_AXIS ] = maxAY;
  acceleration = maxA;
  reset_acceleration_rates();

  return result;
}

int runCalibrateSwitches(tools::Probe& probe, unsigned cycles) {
  s_start();
  return s_end(
    probe,
    calibrateSwitchPositions(probe, cycles) ||
    checkBackSwitchSeparation(probe)
  );
}
