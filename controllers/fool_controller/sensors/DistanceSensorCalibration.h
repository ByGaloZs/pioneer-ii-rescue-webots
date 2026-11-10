#ifndef RESCUE_DISTANCE_SENSOR_CALIBRATION_H_
#define RESCUE_DISTANCE_SENSOR_CALIBRATION_H_

struct DistanceSensorEstimate {
  DistanceSensorEstimate()
      : distanceM(0.0), detected(false), nearSaturated(false) {}

  double distanceM;
  bool detected;
  bool nearSaturated;
};

namespace sensor_calibration {

// Invert the sensor-specific Webots lookup table using piecewise linear
// interpolation. A zero response means no return within the table's range.
DistanceSensorEstimate fromResponse(int sensorIndex, double response);

}  // namespace sensor_calibration

#endif  // RESCUE_DISTANCE_SENSOR_CALIBRATION_H_
