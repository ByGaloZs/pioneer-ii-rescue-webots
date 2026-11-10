#include "DistanceSensorCalibration.h"

#include <cmath>
#include <cstddef>

#include "../config/RobotConfig.h"

namespace sensor_calibration {
namespace {

struct LookupPoint {
  double distanceM;
  double response;
};

const LookupPoint kLaserLookup[] = {
    {0.0, 1020.0}, {0.08, 1020.0}, {0.25, 400.0}, {0.35, 100.0}, {0.4, 0.0}};
const LookupPoint kLongRangeLookup[] = {
    {0.0, 1024.0}, {0.2, 1024.0}, {0.7, 400.0}, {0.9, 100.0}, {1.0, 0.0}};
const LookupPoint kShortRangeLookup[] = {
    {0.0, 1024.0}, {0.2, 1024.0}, {0.4, 0.0}};

}  // namespace

DistanceSensorEstimate fromResponse(int sensorIndex, double response) {
  DistanceSensorEstimate estimate;
  const double maximumRangeM =
      robot_config::distanceSensorMaximumRangeM(sensorIndex);
  estimate.distanceM = maximumRangeM;

  if (!std::isfinite(response) || response <= 0.0)
    return estimate;

  estimate.detected = true;
  if (response >= robot_config::distanceSensorSaturationResponse(sensorIndex)) {
    // The lookup table is flat over its near range, so the exact distance is
    // unavailable when the response saturates.
    estimate.distanceM =
        robot_config::distanceSensorNearSaturationM(sensorIndex);
    estimate.nearSaturated = true;
    return estimate;
  }

  const LookupPoint *lookup;
  std::size_t pointCount;
  if (robot_config::usesFrontLaserLookupProfile(sensorIndex)) {
    lookup = kLaserLookup;
    pointCount = sizeof(kLaserLookup) / sizeof(kLaserLookup[0]);
  } else if (robot_config::isLongRangeDistanceSensor(sensorIndex)) {
    lookup = kLongRangeLookup;
    pointCount = sizeof(kLongRangeLookup) / sizeof(kLongRangeLookup[0]);
  } else {
    lookup = kShortRangeLookup;
    pointCount = sizeof(kShortRangeLookup) / sizeof(kShortRangeLookup[0]);
  }

  for (std::size_t i = 1; i < pointCount; ++i) {
    const LookupPoint &nearPoint = lookup[i - 1];
    const LookupPoint &farPoint = lookup[i];
    if (nearPoint.response > farPoint.response &&
        response <= nearPoint.response && response >= farPoint.response) {
      const double responseFraction =
          (nearPoint.response - response) /
          (nearPoint.response - farPoint.response);
      estimate.distanceM =
          nearPoint.distanceM +
          responseFraction * (farPoint.distanceM - nearPoint.distanceM);
      return estimate;
    }
  }

  // A positive response outside the specified curve can only be interpreted
  // conservatively as a near-saturated return.
  estimate.distanceM = robot_config::distanceSensorNearSaturationM(sensorIndex);
  estimate.nearSaturated = true;
  return estimate;
}

}  // namespace sensor_calibration
