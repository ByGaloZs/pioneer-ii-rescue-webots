#include "MotorController.h"

#include "../config/RobotConfig.h"

#include <limits>

// Guarda el robot; los motores se buscan después en initialize().
MotorController::MotorController(webots::Robot *robot)
    : _robot(robot), _leftMotor(0), _rightMotor(0) {}

void MotorController::initialize() {
  _leftMotor = _robot->getMotor(robot_config::kLeftMotorName);
  _rightMotor = _robot->getMotor(robot_config::kRightMotorName);

  if (_leftMotor != 0 && _rightMotor != 0) {
    // Posición infinita = el motor se controla por velocidad, no por posición.
    const double infinitePosition = std::numeric_limits<double>::infinity();
    _leftMotor->setPosition(infinitePosition);
    _rightMotor->setPosition(infinitePosition);
    stop();
  }
}

void MotorController::setWheelSpeeds(double leftRadPerSec,
                                     double rightRadPerSec) {
  if (_leftMotor == 0 || _rightMotor == 0)
    return;

  // Se recorta cada velocidad al rango [-max, max] para no exceder el motor.
  const double maxSpeed = robot_config::kMaximumWheelSpeedRadPerSec;
  if (leftRadPerSec > maxSpeed) leftRadPerSec = maxSpeed;
  if (leftRadPerSec < -maxSpeed) leftRadPerSec = -maxSpeed;
  if (rightRadPerSec > maxSpeed) rightRadPerSec = maxSpeed;
  if (rightRadPerSec < -maxSpeed) rightRadPerSec = -maxSpeed;

  _leftMotor->setVelocity(leftRadPerSec);
  _rightMotor->setVelocity(rightRadPerSec);
}

void MotorController::stop() {
  setWheelSpeeds(0.0, 0.0);
}
