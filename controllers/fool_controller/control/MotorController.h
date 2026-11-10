#ifndef RESCUE_MOTOR_CONTROLLER_H_
#define RESCUE_MOTOR_CONTROLLER_H_

#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

class MotorController {
public:
  explicit MotorController(webots::Robot *robot);

  void initialize();
  void setWheelSpeeds(double leftRadPerSec, double rightRadPerSec);
  void stop();

private:
  webots::Robot *_robot;
  webots::Motor *_leftMotor;
  webots::Motor *_rightMotor;
};

#endif  // RESCUE_MOTOR_CONTROLLER_H_
