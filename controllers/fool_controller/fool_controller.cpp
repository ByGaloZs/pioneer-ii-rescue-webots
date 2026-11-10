#include <webots/Robot.hpp>

#include <iomanip>
#include <iostream>

#include "control/MotorController.h"
#include "sensors/SensorManager.h"
#include "../../experiments/exp_001_sensor_validation/src/Exp001Logger.h"
#include "../../experiments/exp_002_visual_ranging/src/Exp002Logger.h"

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

  Exp001Logger logger;
  if (!logger.initialize(timeStep, front->getWidth(), front->getHeight(),
                         spherical->getWidth(), spherical->getHeight())) {
    std::cerr << "EXP-001 logging unavailable; sensor diagnostics continue on console."
              << std::endl;
  } else {
    std::cout << "[EXP001_LOG] Writing run data to " << logger.runDirectory()
              << std::endl;
  }

  Exp002Logger visualLogger;
  if (!visualLogger.initialize(timeStep, spherical)) {
    std::cerr << "EXP-002 image logging unavailable; camera diagnostics "
                 "continue on console."
              << std::endl;
  } else {
    std::cout << "[EXP002_LOG] Writing spherical captures to "
              << visualLogger.runDirectory() << std::endl;
  }

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
    if (logger.isOpen()) {
      logger.writeSample(timeS, snapshot, front->getImage() != 0,
                         spherical->getImage() != 0);
    }
    if (visualLogger.isOpen())
      visualLogger.capture(timeS, spherical, snapshot);
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
  logger.finish(robot.getTime());
  visualLogger.finish(robot.getTime());
  sensors.shutdown();
  return 0;
}
