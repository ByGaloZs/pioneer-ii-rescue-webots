#ifndef EXP001_LOGGER_H_
#define EXP001_LOGGER_H_

#include <fstream>
#include <string>

#include "../../../controllers/fool_controller/sensors/SensorSnapshot.h"

class Exp001Logger {
 public:
  Exp001Logger();

  bool initialize(int timeStepMs, int frontWidth, int frontHeight,
                  int sphericalWidth, int sphericalHeight);
  bool isOpen() const;
  const std::string &runDirectory() const;
  void writeSample(double simulationTimeS, const SensorSnapshot &snapshot,
                   bool frontImageAvailable, bool sphericalImageAvailable);
  void finish(double simulationTimeS);

 private:
  std::string _runDirectory;
  std::ofstream _samples;
  std::ofstream _summary;
  bool _initialPoseWritten;
  bool _finished;
  unsigned long _sampleCount;
  double _firstPoseXM;
  double _firstPoseZM;
  double _lastPoseXM;
  double _lastPoseZM;
  bool _hasPose;
};

#endif  // EXP001_LOGGER_H_
