#include "../api/api.h"
#include "../api/diagnostics/diagnostics.h"
#include "../vone/VOne.h"
#include "../../Marlin.h"
#include "../utils/rawToVoltage.h"

#include "processing.h"


static int s_samplePTop(unsigned cycles, unsigned intraSampleDelayMs) {
    const auto maxCycles = 250u;
    PTopPin::Sample samples[maxCycles];

    if (cycles > maxCycles) {
      logWarning
        << F("The requested number of cycles (") << cycles
        << F(") exceeds the maximum (") << maxCycles
        << F("), using maximum")
        << endl;
      cycles = maxCycles;
    }

    // Collect voltage readings
    for (auto i = 0u; i < cycles; ++i) {
      samples[i] = vone->pins.ptop.readValue();
      if (intraSampleDelayMs) {
        delay(intraSampleDelayMs);
      }
    }

    // Output the voltage readings
    log << F("Values -- voltage, startTime, endTime") << endl;
    for (auto i = 0u; i < cycles; ++i) {
      log
        << F("  ") << samples[i].voltage
        << F(",  ") << samples[i].startTime
        << F(",  ") << samples[i].endTime
        << endl;
    }

  return 0;
}

static int s_sampleBedTemperature(unsigned cycles, unsigned intraSampleDelayMs) {
    const auto maxCycles = 250u;
    BedTemperaturePin::Sample samples[maxCycles];

    if (cycles > maxCycles) {
      logWarning
        << F("The requested number of cycles (") << cycles
        << F(") exceeds the maximum (") << maxCycles
        << F("), using maximum")
        << endl;
      cycles = maxCycles;
    }

    // Collect voltage readings
    for (auto i = 0u; i < cycles; ++i) {
      samples[i] = vone->pins.bedTemperature.readValue();
      if (intraSampleDelayMs) {
        delay(intraSampleDelayMs);
      }
    }

    // Output the voltage readings
    log << F("Values -- temperature(C), startTime, endTime") << endl;
    for (auto i = 0u; i < cycles; ++i) {
      log
        << F("  ") << samples[i].temperature
        << F(",  ") << samples[i].startTime
        << F(",  ") << samples[i].endTime
        << endl;
    }

  return 0;
}

static int s_sampleDigitalPin(int pin, unsigned cycles, unsigned intraSampleDelayMs) {
  const auto maxCycles = 250u;
  bool samples[maxCycles];

  if (cycles > maxCycles) {
    logWarning
      << F("The requested number of cycles (") << cycles
      << F(") exceeds the maximum (") << maxCycles
      << F("), using maximum")
      << endl;
    cycles = maxCycles;
  }

  // Collect digital readings
  for (auto i = 0u; i < cycles; ++i) {
    if (pin == P_TOP_PIN) {
      if (vone->pins.ptop.readDigitalValue(samples[i])) {
        logError << F("Unable to complete digital read of p-top pin") << endl;
        return -1;
      }
    } else {
      samples[i] = digitalRead(pin);
    }
    if (intraSampleDelayMs) {
      delay(intraSampleDelayMs);
    }
  }

  // Output the voltage readings
  log << F("Values -- triggered") << endl;
  for (auto i = 0u; i < cycles; ++i) {
    log << F("  ") << samples[i] << endl;
  }

  return 0;
}

int d106_samplePinValues(int pin, unsigned samples, unsigned intraSampleDelayMs) {
  switch (pin) {
    case P_TOP_ANALOG_PIN: return s_samplePTop(samples, intraSampleDelayMs);
    case TEMP_BED_PIN:     return s_sampleBedTemperature(samples, intraSampleDelayMs);
    case P_TOP_PIN:        return s_sampleDigitalPin(pin, samples, intraSampleDelayMs);
    default:
      log
        << F("Ignoring unrecognized pin ") << pin << F(", valid pins are: ")
        << P_TOP_ANALOG_PIN << F(" - p-top analog, ")
        << P_TOP_PIN        << F(" - p-top digital, ")
        << TEMP_BED_PIN     << F(" - bedTemperature")
        << endl;
      return 0;
  }
}

//-------------------------------------------
// Utils/Debugging
int process_dcode(int command_code) {
  auto& tool = vone->toolBox.currentTool();
  switch(command_code) {

    // Enable/disable logging
    case 1:
      // Toggle logging
      logging_enabled = !logging_enabled;
      log
        << F("Logging ")
        << (logging_enabled ? F("ON") : F("OFF"))
        << endl;
      return 0;

    case 2:
      vone->outputStatus();
      return 0;

    case 5: {
      // Stop/resume Stepper
      if (code_seen('E')) {
        const auto enable = code_value() == 1;
        if (enable) {
          log << F("Resuming stepper") << endl;
          vone->stepper.resume();
        } else {
          log << F("Stopping stepper") << endl;
          vone->stepper.stop(F("movement explicitly stopped by D5 command"));
        }
      }

      // Output status
      log
        << F("Stepper is ")
        << (vone->stepper.stopped() ? F("stopped") : F("ready"))
        << endl;

      return 0;
    }

    // Algorithms - prepare to move
    case 101:
      return tool.prepareToMove();

    // Algorithms - Homing
    case 102: {
      bool home_all = !(code_seen('X') || code_seen('Y') || code_seen('Z'));
      if (code_seen('Z')) {
        if (raise()) {
          return -1;
        }
      }
      return rawHome(
        tool,
        home_all || code_seen('X'),
        home_all || code_seen('Y'),
        home_all || code_seen('Z')
      );
    }

    // Algorithms - XY Positioner
    case 103:
      if (tool.prepareToMove()) {
        return -1;
      }

      // Check for move-only flag
      if (code_seen('M')) {
        return moveToXyPositioner(tool);

      } else {
        // Find the center
        float centerX;
        float centerY;
        const long cycles = code_seen('C') ? code_value_long() : defaultXyPositionerCycles;
        const int returnValue = xyPositionerFindCenter(tool, cycles, centerX, centerY);

        // Output
        log
          << F("xyPositionerFindCenter")
          << F(" returnValue:") << returnValue
          << F(" cycles:") << cycles
          << F(" x:") << centerX
          << F(" y:") << centerY
          << endl;
        return 0;
      }

    // Algorithms - Measure probe displacement
    case 104: {
      auto& probe = vone->toolBox.probe;
      if (
        confirmAttached("measure probe displacement", probe) ||
        probe.prepareToMove()
      ) {
        return -1;
      }

      float displacement = 0.0f;
      const int returnValue = measureProbeDisplacement(probe, displacement);

      // Output
      log
        << F("measureProbeDisplacement")
        << F(" returnValue:") << returnValue
        << F(" displacement:") << displacement
        << endl;
      return 0;
    }

    // Algorithms - Measure at switch
    case 105: {
      if (tool.prepareToMove()) {
        return -1;
      }

      if (!code_seen('P')) {
        log << F("No pin provided") << endl;
        return 0;
      }

      const int pin = code_value();
      const auto* endstop = vone->endstops.lookup(pin);
      if (!endstop) {
        log << F("Unknown pin provided: ") << pin << endl;
        return 0;
      }

      const int cycles = code_seen('C') ? code_value() : 1;
      for (int i = 0; i < cycles; ++i) {
        float measurement;
        const int returnValue = measureAtSwitch(*endstop, useDefaultMaxTravel, measurement);

        // Output
        log
          << F("measureAtSwitch")
          << F(" returnValue:") << returnValue
          << F(" switch:") << endstop->name
          << F(" measurement:") << measurement
          << endl;
      }
      return 0;
    }

    // Sample pin values
    case 106: {
      const unsigned pin = code_seen('P') ? code_value() : P_TOP_ANALOG_PIN;
      const unsigned cycles = code_seen('C') ? code_value() : 20;
      const unsigned intraSampleDelayMs = code_seen('M') ? code_value() : 1;
      return d106_samplePinValues(pin, cycles, intraSampleDelayMs);
    }

    // Algorithms - Bed height
    case 107: {
      float x = code_seen('X') ? code_value() : current_position[ X_AXIS ];
      float y = code_seen('Y') ? code_value() : current_position[ Y_AXIS ];
      float height = bedHeightAt(x, y);
      log
        << F("Bed height at (") << x << F(", ") << y << F(") is ") << height
        << endl;
      return 0;
    }

    // Algorithms - measure at switch release
    case 108: {
      if (tool.prepareToMove()) {
        return -1;
      }

      if (!code_seen('P')) {
        log << F("No pin provided") << endl;
        return 0;
      }

      const int pin = code_value();
      const auto* endstop = vone->endstops.lookup(pin);
      if (!endstop) {
        log << F("Unknown pin provided: ") << pin << endl;
        return 0;
      }

      auto axis = endstop->axis;
      auto delay = code_seen('M') ? code_value() : DefaultMeasureAtSwitchReleaseDelay;
      auto startPosition = current_position[axis];
      auto releaseStartedAt = startPosition;
      auto releaseCompletedAt = startPosition;
      auto returnValue = measureAtSwitchRelease(*endstop, releaseStartedAt, releaseCompletedAt, delay);

      // Output
      log
        << F("measureAtSwitchRelease")
        << F(", switch:") << endstop->name
        << F(", delay:") << delay
        << F(", returnValue:") << returnValue
        << F(", releaseStartedAt:") << releaseStartedAt
        << F(", releaseCompletedAt:") << releaseCompletedAt
        << F(", startPosition:") << startPosition
        << endl;

      return 0; // always succeed
    }

    // Algorithms - check back switch separation
    case 109: {
      if (
        !vone->toolBox.dispenser.attached() &&
        !vone->toolBox.probe.attached() &&
        !vone->toolBox.drill.attached()
      ) {
        logError
          << F("Unable to check back switches, current tool, ")
          << tool.name()
          << F(" is not supported for this command")
          << endl;
        return -1;
      }

      return (
        tool.prepareToMove() ||
        checkBackSwitchSeparation(tool)
      );
    }


    // Set rotation speed (without tool prep)
    case 110: {
      auto& drill = vone->toolBox.drill;
      return drill.setRotationSpeed(code_seen('R') ? code_value() : 0.0f);
    }

    // Algorithms - check extents (i.e. volume)
    case 111: {
      const auto tolerance = code_seen('T') ? code_value() : CheckExtents::useDefaultTolerance;
      return checkExtents(tool, tolerance);
    }

    //-------------------------------------------
    // List Commands
    default:
      log << F("D-Commands") << endl;
      log << F("  For manual debugging. subject to change") << endl;
      log << F("General Commands") << endl;
      log << F("  D1 - Toggle logging ON/OFF (default: OFF)") << endl;
      log << F("  D2 - Output status, including switches and stats") << endl;
      log << F("  D5 - stepper stop/resume -- D5 E1 to resume, E0 to stop, no args for status") << endl;
      log << F("") << endl;
      log << F("Algorithms") << endl;
      log << F("  D101 - prepare tool to move") << endl;
      log << F("  D102 - Home -- D102 or D102 XY") << endl;
      log << F("  D103 - xy positioner -- D103 or D103 M (move-only)") << endl;
      log << F("  D104 - measure probe displacement") << endl;
      log << F("  D105 - measure at switch -- D105 P62") << endl;
      log << F("  D106 - sample pin values (P=pin C=cycles M=milliseconds between readings) -- D106 P2 C10 M5 ") << endl;
      log << F("  D108 - measure at switch release -- D108 -Z") << endl;
      log << F("  D109 - check location of xy-positioner's back switch -- D109") << endl;
      log << F("  D110 - set drill rotation speed -- D110 R100, no value or 1 means stop, 0 resets drill") << endl;
      log << F("  D111 - check extents of volume in X and Y -- D111 or D111 T0.001, to override default tolerance") << endl;
      log << F("") << endl;
      return 0;
  }
}
