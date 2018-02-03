#include "diagnostics.h"

#include "../../api/api.h"
#include "../../../Marlin.h"
#include "../../../stepper.h"
#include "../../work/work.h"

#include "../../vone/tools/NullTool.h"

struct Point3d {
  float x, y, z;
};

static int verticalPlane(tools::Tool& tool, const Point3d& a, const Point3d& b, float xySpeed, float zSpeed) {
  return (
    // Ensure we are at the point 1
    moveZ(tool, a.z, zSpeed) || moveXY(tool, a.x, a.y, xySpeed) ||

    // Go to point 2
    moveZ(tool, b.z, zSpeed) || moveXY(tool, b.x, b.y, xySpeed) ||

    // Back to point 1
    moveZ(tool, a.z, zSpeed) || moveXY(tool, a.x, a.y, xySpeed)
  );
}

static int crossPlanes(tools::Tool& tool, const Point3d& a, const Point3d& b, float xySpeed, float zSpeed) {
  const Point3d a2 = { b.x, a.y, a.z };
  const Point3d b2 = { a.x, b.y, b.z };
  if (
    verticalPlane(tool, a, b, xySpeed, zSpeed) ||
    verticalPlane(tool, a2, b2, xySpeed, zSpeed)
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

static int runCrossPlaneSequence(tools::Tool& tool, int steps, const Point3d& a, const Point3d& b, float xySpeed, float zSpeed) {
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

int burnInSequence(tools::NullTool& noTool, int steps) {
  const float bedCenterX = min_pos[X_AXIS] + (max_pos[X_AXIS] - min_pos[X_AXIS]) / 2;
  const float bedCenterY = bedBoundsMinY + (max_pos[Y_AXIS] - bedBoundsMinY) / 2;

  if (
    raise() ||
    confirmAttached("perform burn-in sequence", noTool) ||
    noTool.prepareToMove() ||

    // Move to a known location to start
    moveXY(noTool, 0, 0) ||
    setPosition(0, 0, Z_MAX_POS, 0) ||

    // Entire volume, default (fast) speeds
    // Note: takes about 5min to complete 5 steps
    runCrossPlaneSequence(
      noTool,
      steps,
      { min_pos[X_AXIS], min_pos[Y_AXIS], 1.0f },
      { max_pos[X_AXIS], max_pos[Y_AXIS], Z_MAX_POS - 2 },
      useDefaultFeedrate, useDefaultFeedrate
    ) ||

    // Typical speeds over bed
    // Note: takes about 14min to complete 5 steps
    runCrossPlaneSequence(
      noTool, steps,
      { min_pos[X_AXIS], bedBoundsMinY, 10.0f },
      { max_pos[X_AXIS], max_pos[Y_AXIS], 15.0f },
      500, 100
    ) ||

    // A 2x3" (~50.8x76.2mm) board centered in bed (a typical board) traversed slowly
    // Note: takes about 30min to complete 5 steps
    runCrossPlaneSequence(
      noTool, steps,
      { bedCenterX - 38.1f, bedCenterY - 25.4f, 10.0f },
      { bedCenterX + 38.1f, bedCenterY + 25.4f, 12.0f },
      100, 10
    )
  ) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM("Unable to complete burn-in sequence");
    return -1;
  }
  return 0;
}
