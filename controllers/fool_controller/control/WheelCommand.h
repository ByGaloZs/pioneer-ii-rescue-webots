#ifndef RESCUE_WHEEL_COMMAND_H_
#define RESCUE_WHEEL_COMMAND_H_

// Velocidades de rueda (rad/s) que produce cualquier comportamiento.
struct WheelCommand {
  double leftRadPerSec = 0.0;
  double rightRadPerSec = 0.0;

  static WheelCommand stop() { return WheelCommand(); }
  static WheelCommand of(double left, double right) {
    WheelCommand command;
    command.leftRadPerSec = left;
    command.rightRadPerSec = right;
    return command;
  }
};

#endif  // RESCUE_WHEEL_COMMAND_H_
