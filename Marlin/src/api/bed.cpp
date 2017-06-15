#include <math.h>

#include "../../Marlin.h"
#include "api.h"
#include "tools/tools.h"
#include "../utils/least_squares_fit.h"
#include "../utils/vector_3.h"

class Plane {
public:
  Plane() {}
  Plane(float A, float B, float C, float D) : a(A), b(B), c(C), d(D) {}
  float a=0, b=0, c=0, d=0;
  float heightAt(float x, float y) const {
    return ( a*x + b*y + d) / -c;
  }
};
static Plane s_bedPlane;

bool haveBedMap() {
  return (
    s_bedPlane.a != 0 ||
    s_bedPlane.b != 0 ||
    s_bedPlane.c != 0 ||
    s_bedPlane.d != 0
  );
}

void setBedMap(Plane& plane) {
  s_bedPlane = plane;
}

const Plane& getBedMap() {
  return s_bedPlane;
}

static const float bedBoundsMinY = 40.0f;
bool isOverBed(float x, float y) {
  return (
    x >= min_pos[X_AXIS] &&
    x <= max_pos[X_AXIS] &&
    y >= bedBoundsMinY &&
    y <= max_pos[Y_AXIS]
  );
}

int bedHeightAt(float x, float y, float& z) {
  if (!isOverBed(x, y)) {
    SERIAL_ERROR_START;
    SERIAL_ERRORPGM("Unable to compute bed height, given coordinates ("); SERIAL_ERROR(x);
    SERIAL_ERRORPGM(","); SERIAL_ERROR(y);
    SERIAL_ERRORLNPGM(") fall outside of bed");
    return -1;
  }

  if (!haveBedMap()) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM("Unable to compute bed height, no bed mapping available");
    return -1;
  }

  // Compute Z
  z = getBedMap().heightAt(x, y);
  return 0;
}


void outputBedMap() {
  const Plane& bedPlane = getBedMap();
  SERIAL_PROTOCOLPGM("bedMap a:");  SERIAL_PROTOCOL_F(bedPlane.a, 7);
  SERIAL_PROTOCOLPGM(" b:"); SERIAL_PROTOCOL_F(bedPlane.b, 7);
  SERIAL_PROTOCOLPGM(" c:"); SERIAL_PROTOCOL_F(bedPlane.c, 7);
  SERIAL_PROTOCOLPGM(" d:"); SERIAL_PROTOCOL_F(bedPlane.d, 7);
  SERIAL_PROTOCOLLNPGM("");
}

int mapBed(Tool tool, int gridSize) {
  if (
    raise() ||
    confirmMountedAndNotTriggered("map bed", tool, TOOLS_PROBE) ||
    prepareToolToMove(tool)
  ) {
    return -1;
  }

  if (raise()) {
    return -1;
  }

  struct linear_fit_data lsf_results;
  incremental_LSF_reset(&lsf_results);

  // NOTE: Z-axis motion heats up the z motor, which distorts z travel.
  // As such, choosing an excessively large grid is a bad idea
  const float xStep = X_MAX_LENGTH / (gridSize - 1);
  const float yStep = (Y_MAX_POS - bedBoundsMinY) / (gridSize - 1);
  bool goRight = true; // alternate probing diretion for each row
  float measuredZ;
  for (int i = 0; i < gridSize; ++i) {
    // Compute next y
    const float y = max(bedBoundsMinY, Y_MAX_POS - i * yStep);

    for (int j = 0; j < gridSize; ++j) {
      // Compute next x
      const float factor = goRight ? j : gridSize - 1 - j;
      const float x = max(0, X_MAX_POS - factor * xStep);

      // Move to and measure the current grid position
      if (
        moveXY(tool, x, y) ||
        probe(tool, measuredZ, 1.0f)
      ) {
        return -1;
      }

      // Output measurement
      SERIAL_PROTOCOLPGM("bedMapMeasurement");
      SERIAL_PROTOCOLPGM(" x:"); SERIAL_PROTOCOL_F(x, 3);
      SERIAL_PROTOCOLPGM(" y:"); SERIAL_PROTOCOL_F(y, 3);
      SERIAL_PROTOCOLPGM(" z:"); SERIAL_PROTOCOL_F(measuredZ, 3);
      SERIAL_PROTOCOLPGM("\n");

      // Accumulate
      incremental_LSF(&lsf_results, x, y, measuredZ);
    }

    // Toggle direction of next row
    goRight = !goRight;
  }

  // Compute plane factors
  if (finish_incremental_LSF(&lsf_results)) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM("Unable to compute bed height, no bed mapping available");
    return -1;
  }

  // Update bed map
  Plane plane(
    lsf_results.A,
    lsf_results.B,
    1.0f,
    lsf_results.D
  );
  setBedMap(plane);

  return 0;
}
