#include "../api/api.h"
#include "../../Marlin.h"
#include "../../stepper.h"

struct Point3d {
  float x, y, z;
};

static int rectangle(Tool tool, const Point3d& a, const Point3d& b, float xySpeed, float zSpeed) {
  return (
    // Ensure we are at the point 1
    moveZ(tool, a.z, zSpeed) || moveXY(tool, a.x, a.y, xySpeed) ||

    // Go to point 2
    moveZ(tool, b.z, zSpeed) || moveXY(tool, b.x, b.y, xySpeed) ||

    // Back to point 1
    moveZ(tool, a.z, zSpeed) || moveXY(tool, a.x, a.y, xySpeed)
  );
}

static int crossPlanes(Tool tool, const Point3d& a, const Point3d& b, float xySpeed, float zSpeed) {
  const Point3d a2 = { b.x, a.y, a.z };
  const Point3d b2 = { a.x, b.y, b.z };
  if (
    rectangle(tool, a, b, xySpeed, zSpeed) ||
    rectangle(tool, a2, b2, xySpeed, zSpeed)
  ) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to complete cross-planes from"); SERIAL_ERROR(a.x);
    SERIAL_ERRORPGM(","); SERIAL_ERROR(a.y);
    SERIAL_ERRORPGM(","); SERIAL_ERROR(a.z);
    SERIAL_ERRORPGM(" to "); SERIAL_ERROR(b.x);
    SERIAL_ERRORPGM(","); SERIAL_ERROR(b.y);
    SERIAL_ERRORPGM(","); SERIAL_ERROR(b.z);
    SERIAL_ERRORPGM(" with z-speed "); SERIAL_ERROR(zSpeed);
    SERIAL_ERRORPGM(" and xy-speed "); SERIAL_ERROR(xySpeed);
    SERIAL_EOL;
    return -1;
  }
  return 0;
}

static int runCrossPlaneSequence(Tool tool, int steps, const Point3d& a, const Point3d& b, float xySpeed, float zSpeed) {
  const float xStepSize = (b.x - a.x) / (2 * steps);
  const float yStepSize = (b.y - a.y) / (2 * steps);
  for (int i = 0; i < steps; ++i) {
    {
      // Crosses that shrink in Y as we step
      const float yOffset = i * yStepSize;
      const float y1 = a.y + yOffset;
      const float y2 = b.y - yOffset;
      if (crossPlanes(tool, { a.x, y1, a.z }, { b.x, y2, b.z }, xySpeed, zSpeed)) {
        return -1;
      }
    }

    {
      // Crosses that shrink in X as we step
      const float xOffset = i * xStepSize;
      const float x1 = a.x + xOffset;
      const float x2 = b.x - xOffset;
      if (crossPlanes(tool, { x1, a.y, a.z }, { x2, b.y, b.z }, xySpeed, zSpeed)) {
        return -1;
      }
    }

    refresh_cmd_timeout();
  }
  return 0;
}

int runBurnInSequence(Tool tool, int steps) {
  const float bedCenterX = min_pos[X_AXIS] + (max_pos[X_AXIS] - min_pos[X_AXIS]) / 2;
  const float bedCenterY = bedBoundsMinY + (max_pos[Y_AXIS] - bedBoundsMinY) / 2;

  if (
    raise() ||
    confirmMountedAndNotTriggered("perform burn-in sequence", tool, TOOLS_NONE) ||

    // Move to a known location to start
    moveXY(tool, 0, 0) ||
    setPosition(0, 0, Z_MAX_POS, 0) ||

    // Entire volume, default (fast) speeds
    // Note: takes about 5min to complete 5 steps
    runCrossPlaneSequence(
      tool,
      steps,
      { min_pos[X_AXIS], min_pos[Y_AXIS], 1.0f },
      { max_pos[X_AXIS], max_pos[Y_AXIS], Z_MAX_POS - 2 },
      useDefaultFeedrate, useDefaultFeedrate
    ) ||

    // Typical speeds over bed
    // Note: takes about 14min to complete 5 steps
    runCrossPlaneSequence(
      tool, steps,
      { min_pos[X_AXIS], bedBoundsMinY, 10.0f },
      { max_pos[X_AXIS], max_pos[Y_AXIS], 15.0f },
      500, 100
    ) ||

    // A 2x3" (~50.8x76.2mm) board centered in bed (a typical board) traversed slowly
    // Note: takes about 30min to complete 5 steps
    runCrossPlaneSequence(
      tool, steps,
      { bedCenterX - 38.1f, bedCenterY - 25.4f, 10.0f },
      { bedCenterX + 38.1f, bedCenterY + 25.4f, 12.0f },
      100, 10
    )
  ) {
    overrideLeds(255, 80, 0, 0); // blink yellow
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM("Unable to perform burn-in sequence");
    return -1;
  }

  // Success, pulse leds and go to finished position
  // (trying to make it obvious that we are done)
  raise();
  moveXY(tool, bedCenterX, max_pos[Y_AXIS]);
  st_synchronize();
  overrideLeds(0, 0, 255, 3); // pulse blue

  return 0;
}
