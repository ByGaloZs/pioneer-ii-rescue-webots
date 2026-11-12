#include "SensorManager.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#include "../common/Angles.h"
#include "DistanceSensorCalibration.h"

namespace {

const char *const kDistanceSensorNames[robot_config::kDistanceSensorCount] = {
    "ds0", "ds1", "ds2",  "ds3",  "ds4",  "ds5",  "ds6",  "ds7",
    "ds8", "ds9", "ds10", "ds11", "ds12", "ds13", "ds14", "ds15"};

}  // namespace

SensorManager::SensorManager(webots::Robot *robot)
    : _robot(robot), _timeStep(0), _gps(nullptr), _compass(nullptr),
      _leftEncoder(nullptr), _rightEncoder(nullptr), _distanceSensors(),
      _responseHistory(), _initialized(false) {
  _distanceSensors.fill(nullptr);
}

bool SensorManager::initialize(int timeStep) {
  if (_initialized) return true;
  if (_robot == nullptr || timeStep <= 0) {
    std::cerr << "[SENSORS] Robot o timestep no válidos." << std::endl;
    return false;
  }
  _timeStep = timeStep;

  _gps = _robot->getGPS(robot_config::kGpsName);
  _compass = _robot->getCompass(robot_config::kCompassName);
  _leftEncoder = _robot->getPositionSensor(robot_config::kLeftEncoderName);
  _rightEncoder = _robot->getPositionSensor(robot_config::kRightEncoderName);

  bool ok = _gps && _compass && _leftEncoder && _rightEncoder;
  for (int i = 0; i < robot_config::kDistanceSensorCount; ++i) {
    _distanceSensors[i] = _robot->getDistanceSensor(kDistanceSensorNames[i]);
    ok = ok && _distanceSensors[i] != nullptr;
  }
  if (!ok) {
    std::cerr << "[SENSORS] Falta algún dispositivo (GPS, brújula, encoders o ds0-ds15)."
              << std::endl;
    return false;
  }

  _gps->enable(_timeStep);
  _compass->enable(_timeStep);
  _leftEncoder->enable(_timeStep);
  _rightEncoder->enable(_timeStep);
  for (webots::DistanceSensor *sensor : _distanceSensors) sensor->enable(_timeStep);

  _initialized = true;
  std::cout << "[SENSORS] GPS, brújula, encoders y 16 sensores de distancia activos ("
            << _timeStep << " ms)." << std::endl;
  return true;
}

webots::Camera *SensorManager::enableFrontCamera() {
  if (_frontCamera != nullptr || _robot == nullptr) return _frontCamera;
  _frontCamera = _robot->getCamera(robot_config::kFrontCameraName);
  if (_frontCamera == nullptr) {
    std::cerr << "[SENSORS] No se encontró la cámara frontal." << std::endl;
    return nullptr;
  }
  _frontCamera->enable(_timeStep);
  std::cout << "[SENSORS] Cámara frontal activa: " << _frontCamera->getWidth() << 'x'
            << _frontCamera->getHeight() << ", campo " << _frontCamera->getFov() << " rad"
            << std::endl;
  return _frontCamera;
}

void SensorManager::shutdown() {
  if (!_initialized) return;
  if (_frontCamera != nullptr) _frontCamera->disable();
  _gps->disable();
  _compass->disable();
  _leftEncoder->disable();
  _rightEncoder->disable();
  for (webots::DistanceSensor *sensor : _distanceSensors) sensor->disable();
  _initialized = false;
}

void SensorManager::printDistanceSensorProfiles() const {
  if (!_initialized) return;
  std::cout << "[SENSORS] Perfil de los sensores de distancia:";
  for (int i = 0; i < robot_config::kDistanceSensorCount; ++i) {
    const char *type = "generic";
    switch (_distanceSensors[i]->getType()) {
      case webots::DistanceSensor::LASER: type = "laser"; break;
      case webots::DistanceSensor::INFRA_RED: type = "infra-red"; break;
      case webots::DistanceSensor::SONAR: type = "sonar"; break;
      default: break;
    }
    const int size = _distanceSensors[i]->getLookupTableSize();
    const double *table = _distanceSensors[i]->getLookupTable();
    const double rangeM = (table != nullptr && size > 0) ? table[(size - 1) * 3] : -1.0;
    std::cout << " ds" << i << '=' << type << '/' << std::setprecision(2) << rangeM << 'm';
  }
  std::cout << std::endl;
}

double SensorManager::filteredResponse(int index, double rawResponse) {
  std::deque<double> &history = _responseHistory[index];
  history.push_back(rawResponse);
  while (static_cast<int>(history.size()) > robot_config::kDistanceMedianWindow)
    history.pop_front();
  std::vector<double> sorted(history.begin(), history.end());
  std::sort(sorted.begin(), sorted.end());
  return sorted[sorted.size() / 2];
}

SensorSnapshot SensorManager::read() {
  SensorSnapshot snapshot;
  if (!_initialized) return snapshot;
  snapshot.timeS = _robot->getTime();

  const double *gps = _gps->getValues();
  if (gps != nullptr && std::isfinite(gps[0]) && std::isfinite(gps[2])) {
    snapshot.gpsValid = true;
    snapshot.gpsXM = gps[0];
    snapshot.gpsZM = gps[2];
  }

  const double *compass = _compass->getValues();
  if (compass != nullptr && std::isfinite(compass[0]) && std::isfinite(compass[2]) &&
      std::hypot(compass[0], compass[2]) > 1e-9) {
    snapshot.compassValid = true;
    snapshot.compassYawRad =
        angles::wrap(std::atan2(compass[2], compass[0]) + robot_config::kCompassYawOffsetRad);
  }

  snapshot.leftEncoderRad = _leftEncoder->getValue();
  snapshot.rightEncoderRad = _rightEncoder->getValue();
  snapshot.encodersValid =
      std::isfinite(snapshot.leftEncoderRad) && std::isfinite(snapshot.rightEncoderRad);

  snapshot.distanceSensorsValid = true;
  for (int i = 0; i < robot_config::kDistanceSensorCount; ++i) {
    const double raw = _distanceSensors[i]->getValue();
    if (!std::isfinite(raw)) {
      snapshot.distanceSensorsValid = false;
      continue;
    }
    const double response = filteredResponse(i, raw);
    const DistanceSensorEstimate estimate = sensor_calibration::fromLookupTable(
        _distanceSensors[i]->getLookupTable(), _distanceSensors[i]->getLookupTableSize(),
        response, robot_config::kObstacleReflectionFactor);
    snapshot.distanceResponse[i] = response;
    snapshot.distanceM[i] = estimate.distanceM;
    snapshot.distanceDetected[i] = estimate.detected;
    snapshot.distanceSaturated[i] = estimate.nearSaturated;
  }
  snapshot.computeSectors();
  return snapshot;
}
