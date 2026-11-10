#ifndef RESCUE_SENSOR_SNAPSHOT_H_
#define RESCUE_SENSOR_SNAPSHOT_H_

#include <algorithm>
#include <array>
#include <initializer_list>
#include <limits>

#include "../config/RobotConfig.h"

struct DistanceSensorSectors {
  DistanceSensorSectors()
      : valid(false),
        frontLeft(0.0),
        frontRight(0.0),
        front(0.0),
        diagonalLeft(0.0),
        diagonalRight(0.0),
        leftWall(0.0),
        rightWall(0.0),
        rearLeft(0.0),
        rearRight(0.0) {}

  bool valid;
  // Sector values are maxima of raw Webots responses, not distances in metres.
  double frontLeft;
  double frontRight;
  double front;
  double diagonalLeft;
  double diagonalRight;
  double leftWall;
  double rightWall;
  double rearLeft;
  double rearRight;
};

struct DistanceSectorReading {
  DistanceSectorReading()
      : distanceM(std::numeric_limits<double>::infinity()),
        nearSaturated(false),
        nearSaturationLimitM(0.0) {}

  // Positive infinity means no sensor in this sector returned an obstacle.
  double distanceM;
  bool nearSaturated;
  double nearSaturationLimitM;
};

struct DistanceMetersSectors {
  DistanceSectorReading frontLeft;
  DistanceSectorReading frontRight;
  DistanceSectorReading front;
  DistanceSectorReading diagonalLeft;
  DistanceSectorReading diagonalRight;
  DistanceSectorReading leftWall;
  DistanceSectorReading rightWall;
  DistanceSectorReading rearLeft;
  DistanceSectorReading rearRight;
};

struct SensorSnapshot {
  SensorSnapshot()
      : gpsXM(0.0),
        gpsZM(0.0),
        compassHeadingRad(0.0),
        leftEncoderRad(0.0),
        rightEncoderRad(0.0),
        gpsValid(false),
        compassValid(false),
        encodersValid(false),
        distanceSensorsValid(false),
        distanceSensorValues(),
        distanceMeters(),
        distanceDetected(),
        distanceNearSaturated(),
        distanceSectors(),
        distanceMetersSectors() {}

  double gpsXM;
  double gpsZM;
  double compassHeadingRad;
  double leftEncoderRad;
  double rightEncoderRad;
  bool gpsValid;
  bool compassValid;
  bool encodersValid;
  bool distanceSensorsValid;
  // Raw Webots DistanceSensor response values; conversion to metres requires
  // applying the individual sensor lookup tables and calibration.
  std::array<double, robot_config::kDistanceSensorCount> distanceSensorValues;
  // Inverse-lookup estimates. Undetected sensors hold their maximum range;
  // check distanceDetected before treating that value as an obstacle.
  std::array<double, robot_config::kDistanceSensorCount> distanceMeters;
  std::array<bool, robot_config::kDistanceSensorCount> distanceDetected;
  // True when the response is saturated on the near end (distance <= 0.2 m).
  std::array<bool, robot_config::kDistanceSensorCount> distanceNearSaturated;
  DistanceSensorSectors distanceSectors;
  DistanceMetersSectors distanceMetersSectors;

  void calculateDistanceSensorSectors() {
    if (!distanceSensorsValid)
      return;

    // Sector assignment from practica03/obstacle_avoidance: ds0..ds7 run
    // along the left side from front to rear; ds15..ds8 run along the right.
    const double frontLeftValue =
        std::max(distanceSensorValues[0], distanceSensorValues[1]);
    const double frontRightValue =
        std::max(distanceSensorValues[15], distanceSensorValues[14]);

    distanceSectors.frontLeft = frontLeftValue;
    distanceSectors.frontRight = frontRightValue;
    distanceSectors.front = std::max(frontLeftValue, frontRightValue);
    distanceSectors.diagonalLeft =
        std::max(distanceSensorValues[1], distanceSensorValues[2]);
    distanceSectors.diagonalRight =
        std::max(distanceSensorValues[14], distanceSensorValues[13]);
    distanceSectors.leftWall =
        std::max(distanceSensorValues[2],
                 std::max(distanceSensorValues[3], distanceSensorValues[4]));
    distanceSectors.rightWall =
        std::max(distanceSensorValues[13],
                 std::max(distanceSensorValues[12], distanceSensorValues[11]));
    distanceSectors.rearLeft =
        std::max(distanceSensorValues[5],
                 std::max(distanceSensorValues[6], distanceSensorValues[7]));
    distanceSectors.rearRight =
        std::max(distanceSensorValues[10],
                 std::max(distanceSensorValues[9], distanceSensorValues[8]));
    distanceSectors.valid = true;
  }

  DistanceSectorReading nearestDetectedDistance(
      std::initializer_list<int> sensorIndices) const {
    DistanceSectorReading nearest;
    for (std::initializer_list<int>::const_iterator it = sensorIndices.begin();
         it != sensorIndices.end(); ++it) {
      const int index = *it;
      if (!distanceDetected[index])
        continue;

      if (distanceMeters[index] < nearest.distanceM) {
        nearest.distanceM = distanceMeters[index];
        nearest.nearSaturated = distanceNearSaturated[index];
        nearest.nearSaturationLimitM = distanceNearSaturated[index]
                                           ? robot_config::distanceSensorNearSaturationM(index)
                                           : 0.0;
      } else if (distanceMeters[index] == nearest.distanceM) {
        nearest.nearSaturated =
            nearest.nearSaturated || distanceNearSaturated[index];
        if (distanceNearSaturated[index])
          nearest.nearSaturationLimitM =
              robot_config::distanceSensorNearSaturationM(index);
      }
    }
    return nearest;
  }

  void calculateDistanceMetersSectors() {
    if (!distanceSensorsValid)
      return;

    // Distances use the nearest return in each sector (minimum metres).
    distanceMetersSectors.frontLeft = nearestDetectedDistance({0, 1});
    distanceMetersSectors.frontRight = nearestDetectedDistance({15, 14});
    distanceMetersSectors.front = nearestDetectedDistance({0, 1, 15, 14});
    distanceMetersSectors.diagonalLeft = nearestDetectedDistance({1, 2});
    distanceMetersSectors.diagonalRight = nearestDetectedDistance({14, 13});
    distanceMetersSectors.leftWall = nearestDetectedDistance({2, 3, 4});
    distanceMetersSectors.rightWall = nearestDetectedDistance({13, 12, 11});
    distanceMetersSectors.rearLeft = nearestDetectedDistance({5, 6, 7});
    distanceMetersSectors.rearRight = nearestDetectedDistance({10, 9, 8});
  }
};

#endif  // RESCUE_SENSOR_SNAPSHOT_H_
