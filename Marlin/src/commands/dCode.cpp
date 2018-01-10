#include "../api/api.h"
#include "../vone/VOne.h"
#include "../../Marlin.h"
#include "../utils/rawToVoltage.h"

#include "processing.h"


static int s_samplePTop(unsigned cycles, unsigned intraSampleDelayMs) {
    const auto maxCycles = 250u;
    PTopPin::Sample samples[maxCycles];

    if (cycles > maxCycles) {
      SERIAL_ECHO_START;
      SERIAL_PAIR("Warning: The requested number of cycles (", cycles);
      SERIAL_PAIR(") exceeds the maximum (", maxCycles);
      SERIAL_ECHOLNPGM(") using maximum");
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
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Values -- voltage, startTime, endTime");
    for (auto i = 0u; i < cycles; ++i) {
      SERIAL_PAIR("  ", samples[i].voltage);
      SERIAL_PAIR(",  ", samples[i].startTime);
      SERIAL_PAIR(",  ", samples[i].endTime);
      SERIAL_EOL;
    }

  return 0;
}

static int s_sampleBedTemperature(unsigned cycles, unsigned intraSampleDelayMs) {
    const auto maxCycles = 250u;
    BedTemperaturePin::Sample samples[maxCycles];

    if (cycles > maxCycles) {
      SERIAL_ECHO_START;
      SERIAL_PAIR("Warning: The requested number of cycles (", cycles);
      SERIAL_PAIR(") exceeds the maximum (", maxCycles);
      SERIAL_ECHOLNPGM(") using maximum");
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
    SERIAL_ECHO_START;
    SERIAL_ECHOLNPGM("Values -- temperature(C), startTime, endTime");
    for (auto i = 0u; i < cycles; ++i) {
      SERIAL_PAIR("  ", samples[i].temperature);
      SERIAL_PAIR(",  ", samples[i].startTime);
      SERIAL_PAIR(",  ", samples[i].endTime);
      SERIAL_EOL;
    }

  return 0;
}

static int s_sampleDigitalPin(int pin, unsigned cycles, unsigned intraSampleDelayMs) {
  const auto maxCycles = 250u;
  bool samples[maxCycles];

  if (cycles > maxCycles) {
    SERIAL_ECHO_START;
    SERIAL_PAIR("Warning: The requested number of cycles (", cycles);
    SERIAL_PAIR(") exceeds the maximum (", maxCycles);
    SERIAL_ECHOLNPGM(") using maximum");
    cycles = maxCycles;
  }

  // Collect digital readings
  for (auto i = 0u; i < cycles; ++i) {
    if (pin == P_TOP_PIN) {
      if (vone->pins.ptop.readDigitalValue(samples[i])) {
        SERIAL_ERROR_START;
        SERIAL_ERRORLN("Unable to complete digital read of p-top pin");
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
  SERIAL_ECHO_START;
  SERIAL_ECHOLNPGM("Values -- triggered");
  for (auto i = 0u; i < cycles; ++i) {
    SERIAL_PAIR("  ", samples[i]); SERIAL_EOL;
  }

  return 0;
}

int d106_samplePinValues(int pin, unsigned samples, unsigned intraSampleDelayMs) {
  switch (pin) {
    case P_TOP_ANALOG_PIN: return s_samplePTop(samples, intraSampleDelayMs);
    case TEMP_BED_PIN:     return s_sampleBedTemperature(samples, intraSampleDelayMs);
    case P_TOP_PIN:        return s_sampleDigitalPin(pin, samples, intraSampleDelayMs);
    default:

      SERIAL_ECHO_START;
      SERIAL_PAIR("WARNING: Ignoring unrecognized pin ", pin);
      SERIAL_ECHOPGM(", valid pins are: "); SERIAL_EOL;
      SERIAL_PAIR("    ", P_TOP_ANALOG_PIN ); SERIAL_ECHOPGM(" p-top analog"  ); SERIAL_EOL;
      SERIAL_PAIR("    ", P_TOP_PIN        ); SERIAL_ECHOPGM(" p-top digital" ); SERIAL_EOL;
      SERIAL_PAIR("    ", TEMP_BED_PIN     ); SERIAL_ECHOPGM(" bedTemperature"); SERIAL_EOL;
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
      SERIAL_ECHO_START;
      if (logging_enabled) {
        SERIAL_ECHOPGM("Logging ON");
      } else {
        SERIAL_ECHOPGM("Logging OFF");
      }
      SERIAL_EOL;
      return 0;

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
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM("xyPositionerFindCenter");
        SERIAL_ECHOPGM(" returnValue:"); SERIAL_ECHO(returnValue);
        SERIAL_ECHOPGM(" cycles:"); SERIAL_ECHO(cycles);
        SERIAL_ECHOPGM(" x:"); SERIAL_ECHO(centerX);
        SERIAL_ECHOPGM(" y:"); SERIAL_ECHO(centerY);
        SERIAL_ECHOPGM("\n");
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
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM("measureProbeDisplacement");
      SERIAL_ECHOPGM(" returnValue:"); SERIAL_ECHO(returnValue);
      SERIAL_ECHOPGM(" displacement:"); SERIAL_ECHO(displacement);
      SERIAL_EOL;
      return 0;
    }

    // Algorithms - Measure at switch
    case 105: {
      if (tool.prepareToMove()) {
        return -1;
      }

      const int cycles = code_seen('C') ? code_value() : 1;
      for (int i = 0; i < cycles; ++i) {
        float measurement;
        const int axis = (
          code_seen('X') ? X_AXIS : (
            code_seen('Y') ? Y_AXIS : (
              code_seen('Z') ? Z_AXIS : Z_AXIS
            )
          )
        );
        const int direction = code_prefix() == '-' ? -1 : 1;
        const int returnValue = measureAtSwitch(axis, direction, useDefaultMaxTravel, measurement);

        // Output
        SERIAL_ECHO_START;
        SERIAL_ECHOPGM("measureAtSwitch");
        SERIAL_ECHOPGM(" returnValue:"); SERIAL_ECHO(returnValue);
        SERIAL_ECHOPGM(" axis:"); SERIAL_ECHO(axis_codes[axis]);
        SERIAL_ECHOPGM(" direction:"); SERIAL_ECHO(direction);
        SERIAL_ECHOPGM(" measurement:"); SERIAL_ECHO_F(measurement, 6);
        SERIAL_ECHOLN("");
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
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM("Bed height at ("); SERIAL_ECHO(x);
      SERIAL_ECHOPGM(", "); SERIAL_ECHO(y);
      SERIAL_ECHOPGM(") is "); SERIAL_ECHOLN(height);
      return 0;
    }

    // Algorithms - measure at switch release
    case 108: {
      if (tool.prepareToMove()) {
        return -1;
      }

      const int axis = (
        code_seen('X') ? X_AXIS : (
          code_seen('Y') ? Y_AXIS : (
            code_seen('Z') ? Z_AXIS : Z_AXIS
          )
        )
      );
      const int direction = code_prefix() == '-' ? -1 : 1;
      auto delay = code_seen('M') ? code_value() : DefaultMeasureAtSwitchReleaseDelay;
      auto startPosition = current_position[axis];
      auto releaseStartedAt = startPosition;
      auto releaseCompletedAt = startPosition;
      auto returnValue = measureAtSwitchRelease(axis, direction, releaseStartedAt, releaseCompletedAt, delay);

      // Output
      SERIAL_ECHO_START;
      SERIAL_ECHOPGM("measureAtSwitchRelease");
      SERIAL_PAIR(", axis:", axis_codes[axis]);
      SERIAL_PAIR(", direction:", direction);
      SERIAL_PAIR(", delay:", delay);
      SERIAL_PAIR(", returnValue:", returnValue);
      SERIAL_PAIR(", releaseStartedAt:", releaseStartedAt);
      SERIAL_PAIR(", releaseCompletedAt:", releaseCompletedAt);
      SERIAL_PAIR(", startPosition:", startPosition);
      SERIAL_EOL;

      return 0; // always succeed
    }

    // Set rotation speed (without tool prep)
    case 110: {
      auto& router = vone->toolBox.router;
      return router.setRotationSpeed(code_seen('R') ? code_value() : 0.0f);
    }

    //-------------------------------------------
    // List Commands
    default:
      SERIAL_ECHO_START;
      SERIAL_ECHOLNPGM("D-Commands");
      SERIAL_ECHOLNPGM("  For manual debugging. subject to change");
      SERIAL_ECHOLNPGM("General Commands");
      SERIAL_ECHOLNPGM("  D1 - Toggle logging ON/OFF (default: OFF)");
      SERIAL_ECHOLNPGM("");
      SERIAL_ECHOLNPGM("Algorithms");
      SERIAL_ECHOLNPGM("  D101 - prepare tool to move");
      SERIAL_ECHOLNPGM("  D102 - Home -- D102 or D102 XY");
      SERIAL_ECHOLNPGM("  D103 - xy positioner -- D103 or D103 M (move-only)");
      SERIAL_ECHOLNPGM("  D104 - measure probe displacement");
      SERIAL_ECHOLNPGM("  D105 - measure at switch -- D105 -X");
      SERIAL_ECHOLNPGM("  D106 - sample pin values (P=pin C=cycles M=milliseconds between readings) -- D106 P2 C10 M5 ");
      SERIAL_ECHOLNPGM("  D108 - measure at switch release -- D108 -Z");
      SERIAL_ECHOLNPGM("  D110 - set router rotation speed -- D110 R100, no value or 1 means stop, 0 resets router");
      SERIAL_ECHOLNPGM("");
      return 0;
  }
}
