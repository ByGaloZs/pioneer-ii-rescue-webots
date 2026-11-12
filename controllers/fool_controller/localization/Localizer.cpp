#include "Localizer.h"

#include <cmath>

#include "../common/Angles.h"
#include "../config/RobotConfig.h"

bool Localizer::initialize(const SensorSnapshot &snapshot) {
  if (!snapshot.gpsValid || !snapshot.compassValid || !snapshot.encodersValid) return false;
  _pose.xM = snapshot.gpsXM;
  _pose.zM = snapshot.gpsZM;
  _pose.yawRad = snapshot.compassYawRad;
  _odometry = _pose;
  _encoderYaw = snapshot.compassYawRad;
  _travelledM = 0.0;
  _accumulatedYaw = 0.0;
  _lastLeftRad = snapshot.leftEncoderRad;
  _lastRightRad = snapshot.rightEncoderRad;
  _lastYaw = snapshot.compassYawRad;
  _initialized = true;
  return true;
}

void Localizer::update(const SensorSnapshot &snapshot) {
  if (!_initialized) {
    initialize(snapshot);
    return;
  }

  // 1. Orientación por brújula y giro acumulado.
  double yaw = _lastYaw;
  if (snapshot.compassValid) yaw = snapshot.compassYawRad;
  const double deltaYaw = angles::wrap(yaw - _lastYaw);
  _accumulatedYaw += deltaYaw;
  const double midYaw = angles::wrap(_lastYaw + 0.5 * deltaYaw);
  _lastYaw = yaw;

  // 2. Desplazamiento por encoders.
  if (snapshot.encodersValid) {
    const double dLeft = (snapshot.leftEncoderRad - _lastLeftRad) * robot_config::kWheelRadiusM;
    const double dRight = (snapshot.rightEncoderRad - _lastRightRad) * robot_config::kWheelRadiusM;
    _lastLeftRad = snapshot.leftEncoderRad;
    _lastRightRad = snapshot.rightEncoderRad;

    const double ds = 0.5 * (dLeft + dRight);
    _travelledM += std::fabs(ds);
    // Avance en el mundo: (dx, dz) = (-sin(yaw), -cos(yaw)).
    _odometry.xM += -ds * std::sin(midYaw);
    _odometry.zM += -ds * std::cos(midYaw);
    _odometry.yawRad = yaw;
    // Girar a la izquierda (rueda derecha más rápida) aumenta el yaw.
    _encoderYaw = angles::wrap(_encoderYaw + (dRight - dLeft) / robot_config::kWheelSeparationM);
  }

  // 3. Pose final.
  _pose.yawRad = yaw;
  if (robot_config::kUseGpsForPosition && snapshot.gpsValid) {
    _pose.xM = snapshot.gpsXM;
    _pose.zM = snapshot.gpsZM;
  } else {
    _pose.xM = _odometry.xM;
    _pose.zM = _odometry.zM;
  }
}
