#include <webots/Robot.hpp>

#include <iomanip>
#include <iostream>

#include "control/MotorController.h"
#include "sensors/SensorManager.h"

int main() {
  webots::Robot robot;
  const int timeStep = static_cast<int>(robot.getBasicTimeStep());
  if (timeStep <= 0) {
    std::cerr << "Invalid Webots basic timestep." << std::endl;
    return 1;
  }

  MotorController motors(&robot);
  motors.initialize();
  motors.stop();

  SensorManager sensors(&robot);
  if (!sensors.initialize(timeStep)) {
    std::cerr << "Sensor initialization failed." << std::endl;
    return 1;
  }

  const webots::Camera *front = sensors.forwardCamera();
  const webots::Camera *spherical = sensors.sphericalCamera();
  std::cout << "[STARTUP] Ready. Motors stopped; navigation disabled." << std::endl;
  std::cout << "[CAMERAS] frontal=" << front->getWidth() << 'x' << front->getHeight()
            << " spherical=" << spherical->getWidth() << 'x' << spherical->getHeight()
            << std::endl;

  bool startPoseCaptured = false;
  double nextReportS = 0.0;
  while (robot.step(timeStep) != -1) {
    const SensorSnapshot snapshot = sensors.read();
    if (!startPoseCaptured && snapshot.gpsValid && snapshot.compassValid) {
      startPoseCaptured = true;
      std::cout << std::fixed << std::setprecision(3)
                << "[START] x=" << snapshot.gpsXM
                << " z=" << snapshot.gpsZM
                << " heading_rad=" << snapshot.compassHeadingRad << std::endl;
    }

    const double timeS = robot.getTime();
    if (timeS < nextReportS)
      continue;
    nextReportS = timeS + 1.0;
    std::cout << std::fixed << std::setprecision(3)
              << "[SENSORS] t=" << timeS
              << " gps=" << (snapshot.gpsValid ? "ok" : "invalid")
              << " compass=" << (snapshot.compassValid ? "ok" : "invalid")
              << " encoders=" << (snapshot.encodersValid ? "ok" : "invalid")
              << " range=" << (snapshot.distanceSensorsValid ? "ok" : "invalid");
    if (snapshot.distanceSensorsValid) {
      std::cout << " front_m=" << snapshot.distanceMetersSectors.front.distanceM;
    }
    std::cout << std::endl;
  }

  motors.stop();
  sensors.shutdown();
  return 0;
}
