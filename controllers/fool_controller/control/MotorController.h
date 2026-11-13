#ifndef RESCUE_MOTOR_CONTROLLER_H_
#define RESCUE_MOTOR_CONTROLLER_H_

#include <webots/Motor.hpp>
#include <webots/Robot.hpp>

// Envoltorio de los dos motores de tracción: los pone en modo velocidad y limita lo que se les manda.
class MotorController {
public:
  explicit MotorController(webots::Robot *robot);

  // Obtiene los motores por nombre y los configura en modo velocidad (posición infinita).
  void initialize();
  // Aplica velocidades (rad/s) a cada rueda, recortadas al máximo permitido.
  void setWheelSpeeds(double leftRadPerSec, double rightRadPerSec);
  void stop();

private:
  webots::Robot *_robot;
  webots::Motor *_leftMotor;
  webots::Motor *_rightMotor;
};

#endif  // RESCUE_MOTOR_CONTROLLER_H_
