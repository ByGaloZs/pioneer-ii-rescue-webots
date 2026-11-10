#ifndef EXP002_LOGGER_H_
#define EXP002_LOGGER_H_

#include <fstream>
#include <string>

#include <webots/Camera.hpp>

#include "../../../controllers/fool_controller/sensors/SensorSnapshot.h"

class Exp002Logger {
 public:
  Exp002Logger();

  bool initialize(int timeStepMs, const webots::Camera *sphericalCamera);
  bool isOpen() const;
  const std::string &runDirectory() const;
  void capture(double simulationTimeS, const webots::Camera *sphericalCamera,
               const SensorSnapshot &snapshot);
  void finish(double simulationTimeS);

 private:
  std::string _runDirectory;
  std::ofstream _frames;
  std::ofstream _measurements;
  std::ofstream _summary;
  unsigned long _frameCount;
  unsigned long _savedImageCount;
  bool _finished;
};

#endif  // EXP002_LOGGER_H_
