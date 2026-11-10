#ifndef RESCUE_SENSOR_MANAGER_H_
#define RESCUE_SENSOR_MANAGER_H_

#include <array>

#include <webots/Camera.hpp>
#include <webots/Compass.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/GPS.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

#include "SensorSnapshot.h"

class SensorManager {
public:
  explicit SensorManager(webots::Robot *robot);

  bool initialize(int timeStep);
  void shutdown();
  SensorSnapshot read() const;

  webots::Camera *forwardCamera() const;
  webots::Camera *sphericalCamera() const;

private:
  webots::Robot *_robot;
  int _timeStep;
  webots::GPS *_gps;
  webots::Compass *_compass;
  webots::PositionSensor *_leftEncoder;
  webots::PositionSensor *_rightEncoder;
  std::array<webots::DistanceSensor *, robot_config::kDistanceSensorCount>
      _distanceSensors;
  webots::Camera *_forwardCamera;
  webots::Camera *_sphericalCamera;
  bool _initialized;
};

#endif  // RESCUE_SENSOR_MANAGER_H_
