#include "../../Marlin.h"
#include "../../planner.h"
#include "../api.h"
#include "internal.h"

static int s_alignToReference(const Point2D& reference) {
  // Move the current tool to the reference position
  // i.e. the center of the xy-positioner.
  Point2D toolPosition;
  if (xyPositionerFindCenter(defaultXyPositionerCycles, toolPosition.x, toolPosition.y)) {
    return -1;
  }

  // The tip of the tool is currently at the center of the xy-positioner
  // Override the current position
  current_position[X_AXIS] = reference.x;
  current_position[Y_AXIS] = reference.y;
  plan_set_position(current_position[X_AXIS], current_position[Y_AXIS], current_position[Z_AXIS], current_position[E_AXIS]);

  return 0;
}


int prepareDispenser(const Point2D& reference) {

  // Ensure homed in Z
  if (!homedZ()) {
    if (homeZ()) {
      return -1;
    }
  }

  // Measure the center of the xy-positioner to compensate for the dispenser's offset
  if (s_alignToReference(reference)) {
    return -1;
  }

  if (raise()) {
    return -1;
  }
  return 0;
}
