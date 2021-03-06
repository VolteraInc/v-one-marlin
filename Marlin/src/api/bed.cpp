#include "../api/api.h"
#include "../../Marlin.h"

const float bedBoundsMinY = 40.0f;

// static s_heightMap = [];
bool s_haveBedHeightMap = false;

bool isOverBed(float x, float y) {
  return (
    x >= min_pos[X_AXIS] &&
    x <= max_pos[X_AXIS] &&
    y >= bedBoundsMinY &&
    y <= max_pos[Y_AXIS]
  );
}

bool haveBedHeightMap() {
  return s_haveBedHeightMap;
}

float bedHeightAt(float x, float y) {
  if (!isOverBed(x, y)) {
    return Z_MAX_POS;
  }

  // if (!s_haveBedHeightMap) {
  //   return Z_MAX_POS;
  // }

  log << F("TODO: bed height estimation is not implemented, assuming a height of -1.0mm") << endl;
  return -1.0; // with the current safetyMargin this should
}

// idea: record height at integer coords, will make it trivial to look up nearest point
// e.g. For 3.45,4.60 grab
// fail? if bed is not flat?
// TODO int calibrateBedHeight(Probe& tool, bool saveResult = false) {
//   if (confirmAttachedAndNotTriggered("calibrate bed height", probe)) {
//     return -1;
//   }
//
//   for (;;) {
//     for (;;) {
//
//     }
//   }
//
//   // Success
//   return 0;
// }
