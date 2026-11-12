#include "DistanceSensorCalibration.h"

#include <cmath>

namespace sensor_calibration {

DistanceSensorEstimate fromLookupTable(const double *lookupTable,
                                       int pointCount, double response,
                                       double reflectionFactor) {
  DistanceSensorEstimate estimate;
  if (lookupTable == 0 || pointCount < 2 ||
      !std::isfinite(reflectionFactor) || reflectionFactor <= 0.0 ||
      reflectionFactor > 1.0)
    return estimate;

  const int last = (pointCount - 1) * 3;
  const double maximumRangeM = lookupTable[last];
  if (!std::isfinite(maximumRangeM) || maximumRangeM <= lookupTable[0])
    return estimate;
  estimate.distanceM = maximumRangeM;

  if (!std::isfinite(response) || response <= 0.0)
    return estimate;

  estimate.detected = true;
  const double maximumResponse = lookupTable[1];
  if (!std::isfinite(maximumResponse))
    return DistanceSensorEstimate();

  if (response >= maximumResponse) {
    // A response plateau cannot resolve exact near distance. Report its far
    // boundary as the saturation limit, using the active device's table.
    estimate.nearSaturated = true;
    estimate.nearSaturationLimitM = lookupTable[0] * reflectionFactor;
    for (int i = 1; i < pointCount; ++i) {
      const int current = i * 3;
      if (!std::isfinite(lookupTable[current]) ||
          !std::isfinite(lookupTable[current + 1]))
        return DistanceSensorEstimate();
      if (lookupTable[current + 1] != maximumResponse)
        break;
      estimate.nearSaturationLimitM = lookupTable[current] * reflectionFactor;
    }
    estimate.distanceM = estimate.nearSaturationLimitM;
    return estimate;
  }

  for (int i = 1; i < pointCount; ++i) {
    const int nearIndex = (i - 1) * 3;
    const int farIndex = i * 3;
    const double nearDistance = lookupTable[nearIndex];
    const double farDistance = lookupTable[farIndex];
    const double nearResponse = lookupTable[nearIndex + 1];
    const double farResponse = lookupTable[farIndex + 1];
    if (!std::isfinite(nearDistance) || !std::isfinite(farDistance) ||
        !std::isfinite(nearResponse) || !std::isfinite(farResponse) ||
        farDistance <= nearDistance)
      return DistanceSensorEstimate();

    if (nearResponse > farResponse && response <= nearResponse &&
        response >= farResponse) {
      const double fraction =
          (nearResponse - response) / (nearResponse - farResponse);
      estimate.distanceM =
          (nearDistance + fraction * (farDistance - nearDistance)) *
          reflectionFactor;
      return estimate;
    }
  }

  // Positive values beyond the table's maximum are near-saturated.
  estimate.nearSaturated = true;
  estimate.nearSaturationLimitM = lookupTable[0] * reflectionFactor;
  estimate.distanceM = estimate.nearSaturationLimitM;
  return estimate;
}

}  // namespace sensor_calibration
