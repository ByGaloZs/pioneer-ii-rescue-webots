#include "SensorManager.h"

#include <cmath>
#include <iostream>

#include "../config/RobotConfig.h"
#include "DistanceSensorCalibration.h"

namespace {

const char *const kDistanceSensorNames[robot_config::kDistanceSensorCount] = {
    "ds0", "ds1", "ds2", "ds3", "ds4", "ds5", "ds6", "ds7",
    "ds8", "ds9", "ds10", "ds11", "ds12", "ds13", "ds14", "ds15"};

}  // namespace

SensorManager::SensorManager(webots::Robot *robot)
    : _robot(robot),
      _timeStep(0),
      _gps(0),
      _compass(0),
      _leftEncoder(0),
      _rightEncoder(0),
      _distanceSensors(),
      _forwardCamera(0),
      _sphericalCamera(0),
      _initialized(false) {}

bool SensorManager::initialize(int timeStep) {
  if (_robot == 0 || timeStep <= 0) {
    std::cerr << "SensorManager: robot or timestep is invalid." << std::endl;
    return false;
  }

  if (_initialized)
    return true;

  _timeStep = timeStep;

  _gps = _robot->getGPS(robot_config::kGpsName);
  _compass = _robot->getCompass(robot_config::kCompassName);
  _leftEncoder = _robot->getPositionSensor(robot_config::kLeftEncoderName);
  _rightEncoder = _robot->getPositionSensor(robot_config::kRightEncoderName);
  _forwardCamera = _robot->getCamera(robot_config::kFrontCameraName);
  _sphericalCamera = _robot->getCamera(robot_config::kSphericalCameraName);

  bool devicesFound = true;
  if (_gps == 0) {
    std::cerr << "SensorManager: missing GPS '" << robot_config::kGpsName
              << "'." << std::endl;
    devicesFound = false;
  }
  if (_compass == 0) {
    std::cerr << "SensorManager: missing compass '"
              << robot_config::kCompassName << "'." << std::endl;
    devicesFound = false;
  }
  if (_leftEncoder == 0) {
    std::cerr << "SensorManager: missing position sensor '"
              << robot_config::kLeftEncoderName << "'." << std::endl;
    devicesFound = false;
  }
  if (_rightEncoder == 0) {
    std::cerr << "SensorManager: missing position sensor '"
              << robot_config::kRightEncoderName << "'." << std::endl;
    devicesFound = false;
  }
  if (_forwardCamera == 0) {
    std::cerr << "SensorManager: missing camera '"
              << robot_config::kFrontCameraName << "'." << std::endl;
    devicesFound = false;
  }
  if (_sphericalCamera == 0) {
    std::cerr << "SensorManager: missing camera '"
              << robot_config::kSphericalCameraName << "'." << std::endl;
    devicesFound = false;
  }

  for (int i = 0; i < robot_config::kDistanceSensorCount; ++i) {
    _distanceSensors[i] = _robot->getDistanceSensor(kDistanceSensorNames[i]);
    if (_distanceSensors[i] == 0) {
      std::cerr << "SensorManager: missing distance sensor '"
                << kDistanceSensorNames[i] << "'." << std::endl;
      devicesFound = false;
    }
  }

  if (!devicesFound) {
    _gps = 0;
    _compass = 0;
    _leftEncoder = 0;
    _rightEncoder = 0;
    _forwardCamera = 0;
    _sphericalCamera = 0;
    _distanceSensors.fill(0);
    return false;
  }

  _gps->enable(_timeStep);
  _compass->enable(_timeStep);
  _leftEncoder->enable(_timeStep);
  _rightEncoder->enable(_timeStep);
  _forwardCamera->enable(_timeStep);
  _sphericalCamera->enable(_timeStep);
  for (int i = 0; i < robot_config::kDistanceSensorCount; ++i)
    _distanceSensors[i]->enable(_timeStep);

  _initialized = true;
  std::cout << "SensorManager: initialized GPS, compass, wheel encoders, "
            << robot_config::kDistanceSensorCount
            << " distance sensors, and both cameras." << std::endl;
  std::cout << "SensorManager: distance sensor profiles:";
  for (int i = 0; i < robot_config::kDistanceSensorCount; ++i) {
    const char *type = "generic";
    switch (_distanceSensors[i]->getType()) {
      case webots::DistanceSensor::LASER:
        type = "laser";
        break;
      case webots::DistanceSensor::INFRA_RED:
        type = "infra-red";
        break;
      case webots::DistanceSensor::SONAR:
        type = "sonar";
        break;
      case webots::DistanceSensor::GENERIC:
        type = "generic";
        break;
    }
    const int lookupSize = _distanceSensors[i]->getLookupTableSize();
    const double *lookup = _distanceSensors[i]->getLookupTable();
    const double maximumRangeM =
        lookup != 0 && lookupSize > 0 ? lookup[(lookupSize - 1) * 3] : -1.0;
    std::cout << " ds" << i << '=' << type << ':' << maximumRangeM << 'm';
  }
  std::cout << std::endl;
  return true;
}

void SensorManager::shutdown() {
  if (!_initialized)
    return;

  _gps->disable();
  _compass->disable();
  _leftEncoder->disable();
  _rightEncoder->disable();
  _forwardCamera->disable();
  _sphericalCamera->disable();
  for (int i = 0; i < robot_config::kDistanceSensorCount; ++i)
    _distanceSensors[i]->disable();

  _initialized = false;
}

SensorSnapshot SensorManager::read() const {
  SensorSnapshot snapshot;
  if (!_initialized)
    return snapshot;

  const double *gpsValues = _gps->getValues();
  if (gpsValues != 0) {
    snapshot.gpsXM = gpsValues[0];
    snapshot.gpsZM = gpsValues[2];
    snapshot.gpsValid = std::isfinite(snapshot.gpsXM) && std::isfinite(snapshot.gpsZM);
  }

  const double *compassValues = _compass->getValues();
  if (compassValues != 0) {
    // Match the compass variation convention validated in practice 04.
    snapshot.compassHeadingRad =
        std::atan2(compassValues[2], compassValues[0]);
    snapshot.compassValid = std::isfinite(compassValues[0]) &&
                            std::isfinite(compassValues[2]) &&
                            std::hypot(compassValues[0], compassValues[2]) > 1e-9;
  }

  snapshot.leftEncoderRad = _leftEncoder->getValue();
  snapshot.rightEncoderRad = _rightEncoder->getValue();
  snapshot.encodersValid = std::isfinite(snapshot.leftEncoderRad) &&
                           std::isfinite(snapshot.rightEncoderRad);

  snapshot.distanceSensorsValid = true;
  for (int i = 0; i < robot_config::kDistanceSensorCount; ++i) {
    if (_distanceSensors[i] == 0) {
      snapshot.distanceSensorsValid = false;
      break;
    }
    const double response = _distanceSensors[i]->getValue();
    if (!std::isfinite(response)) {
      snapshot.distanceSensorsValid = false;
      break;
    }
    snapshot.distanceSensorValues[i] = response;
    const DistanceSensorEstimate estimate =
        sensor_calibration::fromResponse(i, response);
    snapshot.distanceMeters[i] = estimate.distanceM;
    snapshot.distanceDetected[i] = estimate.detected;
    snapshot.distanceNearSaturated[i] = estimate.nearSaturated;
  }
  snapshot.calculateDistanceSensorSectors();
  snapshot.calculateDistanceMetersSectors();

  return snapshot;
}

webots::Camera *SensorManager::forwardCamera() const {
  return _initialized ? _forwardCamera : 0;
}

webots::Camera *SensorManager::sphericalCamera() const {
  return _initialized ? _sphericalCamera : 0;
}
