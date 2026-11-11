#include <webots/Robot.hpp>

#include <algorithm>
#include <cctype>
#include <iomanip>
#include <iostream>
#include <string>

#include "../fool_controller/control/MotorController.h"
#include "../fool_controller/sensors/SensorManager.h"
#include "../../experiments/exp_002_visual_ranging/src/Exp002Logger.h"

namespace {

bool parseExperimentArgs(int argc, const char *argv[], std::string *trialId) {
  const std::string prefix = "experiment=002;trial=";
  if (argc != 2)
    return false;
  const std::string argument(argv[1]);
  if (argument.compare(0, prefix.size(), prefix) != 0)
    return false;
  *trialId = argument.substr(prefix.size());
  if (trialId->empty() || trialId->size() > 64)
    return false;
  return std::all_of(trialId->begin(), trialId->end(), [](unsigned char value) {
    return std::isalnum(value) || value == '_' || value == '-';
  });
}

}  // namespace

int main(int argc, const char *argv[]) {
  std::string trialId;
  if (!parseExperimentArgs(argc, argv, &trialId)) {
    std::cerr << "Experiment test controller requires one valid "
                 "experiment=002;trial=<label> argument."
              << std::endl;
    return 1;
  }

  const bool calibrationTrial = trialId.compare(0, 8, "calib01_") == 0;
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
  std::cout << "[STARTUP] EXP-002 test controller ready. Motors stopped."
            << std::endl;
  std::cout << "[CAMERAS] frontal=" << front->getWidth() << 'x'
            << front->getHeight() << " spherical=" << spherical->getWidth()
            << 'x' << spherical->getHeight() << std::endl;

  Exp002Logger visualLogger;
  if (!visualLogger.initialize(timeStep, spherical, trialId, calibrationTrial)) {
    std::cerr << "EXP-002 image logging unavailable." << std::endl;
    sensors.shutdown();
    return 1;
  }
  std::cout << "[EXP002_LOG] Writing captures to "
            << visualLogger.runDirectory() << std::endl;

  bool startPoseCaptured = false;
  double nextReportS = 0.0;
  double nextCaptureS = calibrationTrial ? 1.0 : 0.0;
  while (robot.step(timeStep) != -1) {
    const SensorSnapshot snapshot = sensors.read();
    if (!startPoseCaptured && snapshot.gpsValid && snapshot.compassValid) {
      startPoseCaptured = true;
      std::cout << std::fixed << std::setprecision(3)
                << "[START] x=" << snapshot.gpsXM << " z=" << snapshot.gpsZM
                << " heading_rad=" << snapshot.compassHeadingRad << std::endl;
    }

    const double timeS = robot.getTime();
    if (visualLogger.isOpen() && timeS + 1e-9 >= nextCaptureS) {
      const bool saved = visualLogger.capture(timeS, spherical, snapshot);
      nextCaptureS = timeS + 1.0;
      std::cout << "[EXP002_FRAME] trial=" << trialId
                << " t=" << std::fixed << std::setprecision(3) << timeS
                << " saved=" << (saved ? 1 : 0) << std::endl;
      if (calibrationTrial && saved) {
        visualLogger.finish(timeS);
        std::cout << "[EXP002_DONE] trial=" << trialId
                  << " image saved and files finalized." << std::endl;
      }
    }

    if (timeS < nextReportS)
      continue;
    nextReportS = timeS + 1.0;
    std::cout << std::fixed << std::setprecision(3) << "[SENSORS] t=" << timeS
              << " gps=" << (snapshot.gpsValid ? "ok" : "invalid")
              << " compass=" << (snapshot.compassValid ? "ok" : "invalid")
              << " encoders=" << (snapshot.encodersValid ? "ok" : "invalid")
              << " range="
              << (snapshot.distanceSensorsValid ? "ok" : "invalid");
    if (snapshot.distanceSensorsValid)
      std::cout << " front_m="
                << snapshot.distanceMetersSectors.front.distanceM;
    std::cout << std::endl;
  }

  motors.stop();
  visualLogger.finish(robot.getTime());
  sensors.shutdown();
  return 0;
}
