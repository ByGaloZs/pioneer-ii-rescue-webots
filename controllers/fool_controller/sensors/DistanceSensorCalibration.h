#ifndef RESCUE_DISTANCE_SENSOR_CALIBRATION_H_
#define RESCUE_DISTANCE_SENSOR_CALIBRATION_H_

struct DistanceSensorEstimate {
  DistanceSensorEstimate()
      : distanceM(0.0), detected(false), nearSaturated(false),
        nearSaturationLimitM(0.0) {}

  double distanceM;
  bool detected;
  bool nearSaturated;
  double nearSaturationLimitM;
};

namespace sensor_calibration {

// Invert the active Webots [distance, response, noise] table, then scale the
// effective distance by the assumed IR reflection factor. This factor is
// material-specific; it is not inferred from the sensor response.
// pointCount is the row count from DistanceSensor::getLookupTableSize.
DistanceSensorEstimate fromLookupTable(const double *lookupTable, int pointCount,
                                      double response,
                                      double reflectionFactor);

}  // namespace sensor_calibration

#endif  // RESCUE_DISTANCE_SENSOR_CALIBRATION_H_
