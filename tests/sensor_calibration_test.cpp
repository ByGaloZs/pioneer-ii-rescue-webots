#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

#include "../controllers/fool_controller/sensors/DistanceSensorCalibration.h"
#include "../controllers/fool_controller/sensors/SensorSnapshot.h"

namespace {

void expectNear(double actual, double expected, double tolerance = 1e-9) {
  assert(std::fabs(actual - expected) <= tolerance);
}

}  // namespace

int main() {
  using sensor_calibration::fromResponse;

  const DistanceSensorEstimate laserTableKnot = fromResponse(0, 400.0);
  assert(laserTableKnot.detected);
  assert(!laserTableKnot.nearSaturated);
  expectNear(laserTableKnot.distanceM, 0.25);

  const DistanceSensorEstimate laserTableInterpolation =
      fromResponse(15, 300.0);
  assert(laserTableInterpolation.detected);
  expectNear(laserTableInterpolation.distanceM, 0.25 + (100.0 / 300.0) * 0.1);

  const DistanceSensorEstimate longInfraredTableKnot = fromResponse(3, 400.0);
  assert(longInfraredTableKnot.detected);
  expectNear(longInfraredTableKnot.distanceM, 0.7);

  const DistanceSensorEstimate shortTableInterpolation = fromResponse(4, 512.0);
  assert(shortTableInterpolation.detected);
  expectNear(shortTableInterpolation.distanceM, 0.3);

  const DistanceSensorEstimate noLongRangeReturn = fromResponse(3, 0.0);
  assert(!noLongRangeReturn.detected);
  expectNear(noLongRangeReturn.distanceM, 1.0);

  const DistanceSensorEstimate noShortRangeReturn = fromResponse(4, 0.0);
  assert(!noShortRangeReturn.detected);
  expectNear(noShortRangeReturn.distanceM, 0.4);

  const DistanceSensorEstimate laserNearSaturation = fromResponse(0, 1107.0);
  assert(laserNearSaturation.detected);
  assert(laserNearSaturation.nearSaturated);
  expectNear(laserNearSaturation.distanceM, 0.08);

  const DistanceSensorEstimate shortRangeNearSaturation =
      fromResponse(8, 1107.0);
  assert(shortRangeNearSaturation.detected);
  assert(shortRangeNearSaturation.nearSaturated);
  expectNear(shortRangeNearSaturation.distanceM, 0.2);

  const DistanceSensorEstimate invalidReading =
      fromResponse(3, std::numeric_limits<double>::quiet_NaN());
  assert(!invalidReading.detected);
  expectNear(invalidReading.distanceM, 1.0);

  SensorSnapshot snapshot;
  snapshot.distanceSensorsValid = true;
  snapshot.distanceSensorValues[0] = 250.0;
  snapshot.distanceSensorValues[1] = 500.0;
  snapshot.distanceSensorValues[15] = 300.0;
  snapshot.distanceDetected[0] = true;
  snapshot.distanceDetected[1] = true;
  snapshot.distanceDetected[15] = true;
  snapshot.distanceMeters[0] = 0.45;
  snapshot.distanceMeters[1] = 0.08;
  snapshot.distanceMeters[15] = 0.65;
  snapshot.distanceNearSaturated[1] = true;
  snapshot.calculateDistanceSensorSectors();
  snapshot.calculateDistanceMetersSectors();
  expectNear(snapshot.distanceSectors.front, 500.0);
  expectNear(snapshot.distanceMetersSectors.front.distanceM, 0.08);
  assert(snapshot.distanceMetersSectors.front.nearSaturated);
  expectNear(snapshot.distanceMetersSectors.front.nearSaturationLimitM, 0.08);

  std::cout << "Sensor calibration and sector tests passed." << std::endl;
  return 0;
}
