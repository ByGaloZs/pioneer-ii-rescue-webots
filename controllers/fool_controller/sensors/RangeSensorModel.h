#ifndef RESCUE_RANGE_SENSOR_MODEL_H_
#define RESCUE_RANGE_SENSOR_MODEL_H_

// Local X/Z position and Y-axis rotation from Pioneer2_Robot.proto.
// Webots DistanceSensor rays point along their local +X axis.
struct RangeSensorMount {
  double xM;
  double zM;
  double rotationRad;
};

inline RangeSensorMount rangeSensorMount(int index) {
  static const RangeSensorMount mounts[] = {
      {-0.027, -0.164, 1.745}, {-0.077, -0.147, 2.094},
      {-0.118, -0.116, 2.443}, {-0.136, -0.071, 3.14},
      {-0.136, 0.16, -3.14}, {-0.118, 0.205, -2.443},
      {-0.077, 0.236, -2.094}, {-0.027, 0.253, -1.745},
      {0.027, 0.253, -1.396}, {0.077, 0.236, -1.047},
      {0.118, 0.205, -0.698}, {0.136, 0.16, 0.0},
      {0.136, -0.071, 0.0}, {0.118, -0.116, 0.698},
      {0.077, -0.147, 1.047}, {0.027, -0.164, 1.396}};
  return mounts[index];
}

#endif  // RESCUE_RANGE_SENSOR_MODEL_H_
