#include <cassert>
#include <fstream>
#include <sstream>
#include <string>

#include "../experiments/exp_001_sensor_validation/src/Exp001Logger.h"

int main() {
  Exp001Logger logger;
  assert(logger.initialize(32, 640, 480, 1024, 512));
  assert(logger.isOpen());

  SensorSnapshot snapshot;
  snapshot.gpsValid = true;
  snapshot.gpsXM = 1.25;
  snapshot.gpsZM = -5.6;
  snapshot.compassValid = true;
  snapshot.compassHeadingRad = 3.14159;
  snapshot.encodersValid = true;
  snapshot.leftEncoderRad = 0.0;
  snapshot.rightEncoderRad = 0.0;
  snapshot.distanceSensorsValid = true;
  snapshot.distanceSensorValues[0] = 512.0;
  snapshot.distanceMeters[0] = 0.42;
  snapshot.distanceDetected[0] = true;
  snapshot.distanceNearSaturated[0] = false;

  logger.writeSample(1.0, snapshot, true, true);
  snapshot.gpsXM = 1.25;
  snapshot.gpsZM = -5.6;
  logger.writeSample(2.0, snapshot, true, true);
  logger.finish(2.0);
  assert(!logger.isOpen());

  std::ifstream csv((logger.runDirectory() + "/sensor_samples.csv").c_str());
  assert(csv.good());
  std::string header;
  std::string firstRow;
  std::string secondRow;
  std::getline(csv, header);
  std::getline(csv, firstRow);
  std::getline(csv, secondRow);
  assert(header.find("ds15_near_saturated") != std::string::npos);
  assert(firstRow.find("1.000000,1,1.250000,-5.600000") == 0);
  assert(secondRow.find("2.000000,1,1.250000,-5.600000") == 0);
  assert(secondRow.find(",512.000000,0.420000,1,0,") != std::string::npos);

  std::ifstream summary((logger.runDirectory() + "/summary.txt").c_str());
  assert(summary.good());
  std::ostringstream summaryText;
  summaryText << summary.rdbuf();
  assert(summaryText.str().find("samples_written=2") != std::string::npos);
  assert(summaryText.str().find("gps_displacement_m=0.000000") !=
         std::string::npos);

  Exp001Logger secondRun;
  assert(secondRun.initialize(32, 640, 480, 1024, 512));
  assert(secondRun.runDirectory() != logger.runDirectory());
  secondRun.finish(0.0);
  return 0;
}
