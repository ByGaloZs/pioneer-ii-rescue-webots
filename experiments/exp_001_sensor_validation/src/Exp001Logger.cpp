#include "Exp001Logger.h"

#include <cerrno>
#include <cmath>
#include <ctime>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "../../../controllers/fool_controller/config/RobotConfig.h"

#ifdef _WIN32
#include <direct.h>
#include <sys/stat.h>
#else
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace {

bool makeDirectory(const std::string &path) {
#ifdef _WIN32
  if (_mkdir(path.c_str()) == 0)
#else
  if (mkdir(path.c_str(), 0775) == 0)
#endif
    return true;
  return false;
}

bool isDirectory(const std::string &path) {
  struct stat information;
  if (stat(path.c_str(), &information) != 0)
    return false;
#ifdef _WIN32
  return (information.st_mode & _S_IFDIR) != 0;
#else
  return S_ISDIR(information.st_mode);
#endif
}

std::string findResultsRoot() {
  const char *configuredRoot = std::getenv("EXP001_RESULTS_ROOT");
  if (configuredRoot != 0 && configuredRoot[0] != '\0')
    return configuredRoot;

  char workingDirectory[4096];
#ifdef _WIN32
  if (_getcwd(workingDirectory, sizeof(workingDirectory)) == 0)
#else
  if (getcwd(workingDirectory, sizeof(workingDirectory)) == 0)
#endif
    return std::string();

  std::string ancestor(workingDirectory);
  const std::string relativeResults =
      "experiments/exp_001_sensor_validation/results";
  while (!ancestor.empty()) {
    const std::string candidate = ancestor + '/' + relativeResults;
    if (isDirectory(candidate))
      return candidate;

    const std::string::size_type separator = ancestor.find_last_of('/');
    if (separator == std::string::npos)
      break;
    if (separator == 0) {
      ancestor = "/";
      if (isDirectory(ancestor + relativeResults))
        return ancestor + relativeResults;
      break;
    }
    ancestor.erase(separator);
  }
  return std::string();
}

std::string timestampForDirectory() {
  const std::time_t now = std::time(0);
  std::tm localTime;
#ifdef _WIN32
  localtime_s(&localTime, &now);
#else
  localtime_r(&now, &localTime);
#endif

  std::ostringstream name;
  name << "run_" << std::put_time(&localTime, "%Y%m%d_%H%M%S");
  return name.str();
}

void writeBoolean(std::ostream &stream, bool value) {
  stream << (value ? 1 : 0);
}

}  // namespace

Exp001Logger::Exp001Logger()
    : _runDirectory(),
      _samples(),
      _summary(),
      _initialPoseWritten(false),
      _finished(false),
      _sampleCount(0),
      _firstPoseXM(0.0),
      _firstPoseZM(0.0),
      _lastPoseXM(0.0),
      _lastPoseZM(0.0),
      _hasPose(false) {}

bool Exp001Logger::initialize(int timeStepMs, int frontWidth, int frontHeight,
                              int sphericalWidth, int sphericalHeight) {
  const std::string resultsRoot = findResultsRoot();
  if (resultsRoot.empty() ||
      (!isDirectory(resultsRoot) && !makeDirectory(resultsRoot) &&
       !isDirectory(resultsRoot))) {
    std::cerr << "EXP-001 logger: cannot access results directory '"
              << (resultsRoot.empty() ? "<project root not found>" : resultsRoot)
              << "'. Start Webots from within the project or set "
                 "EXP001_RESULTS_ROOT."
              << std::endl;
    return false;
  }

  const std::string runName = timestampForDirectory();
  for (unsigned int suffix = 0; suffix < 1000; ++suffix) {
    std::ostringstream candidate;
    candidate << resultsRoot << '/' << runName;
    if (suffix > 0)
      candidate << '_' << suffix;
    if (makeDirectory(candidate.str())) {
      _runDirectory = candidate.str();
      break;
    }
  }
  if (_runDirectory.empty()) {
    std::cerr << "EXP-001 logger: cannot create a unique run directory under '"
              << resultsRoot << "'." << std::endl;
    return false;
  }

  _samples.open((_runDirectory + "/sensor_samples.csv").c_str());
  _summary.open((_runDirectory + "/summary.txt").c_str());
  if (!_samples || !_summary) {
    std::cerr << "EXP-001 logger: cannot create output files in '"
              << _runDirectory << "'." << std::endl;
    return false;
  }

  _samples << "simulation_time_s,gps_valid,gps_x_m,gps_z_m,compass_valid,"
              "heading_rad,encoders_valid,left_encoder_rad,right_encoder_rad,"
              "distance_sensors_valid,front_camera_image_available,"
              "spherical_camera_image_available,gps_delta_x_m,gps_delta_z_m,"
              "gps_displacement_m";
  for (int i = 0; i < robot_config::kDistanceSensorCount; ++i)
    _samples << ",ds" << i << "_raw,ds" << i << "_distance_m,ds" << i
             << "_detected,ds" << i << "_near_saturated";
  _samples << '\n' << std::fixed << std::setprecision(6);

  _summary << "experiment_id=EXP-001\n"
           << "run_directory=" << _runDirectory << '\n'
           << "scenario=not_provided_to_controller\n"
           << "world_path=not_read_by_controller\n"
           << "webots_timestep_ms=" << timeStepMs << '\n'
           << "front_camera_resolution=" << frontWidth << 'x' << frontHeight
           << '\n'
           << "spherical_camera_resolution=" << sphericalWidth << 'x'
           << sphericalHeight << '\n'
           << "motor_stop_commanded=true\n"
           << "sample_period_simulated_s=1\n";
  _summary.flush();
  _samples.flush();
  return true;
}

bool Exp001Logger::isOpen() const {
  return _samples.is_open() && _summary.is_open();
}

const std::string &Exp001Logger::runDirectory() const {
  return _runDirectory;
}

void Exp001Logger::writeSample(double simulationTimeS,
                               const SensorSnapshot &snapshot,
                               bool frontImageAvailable,
                               bool sphericalImageAvailable) {
  if (!isOpen() || _finished)
    return;

  const bool poseValid = snapshot.gpsValid && std::isfinite(snapshot.gpsXM) &&
                         std::isfinite(snapshot.gpsZM);
  if (poseValid) {
    if (!_hasPose) {
      _firstPoseXM = snapshot.gpsXM;
      _firstPoseZM = snapshot.gpsZM;
      _hasPose = true;
    }
    _lastPoseXM = snapshot.gpsXM;
    _lastPoseZM = snapshot.gpsZM;
    if (!_initialPoseWritten) {
      _summary << std::fixed << std::setprecision(6)
               << "initial_pose_x_m="
               << _firstPoseXM << '\n'
               << "initial_pose_z_m=" << _firstPoseZM << '\n';
      _initialPoseWritten = true;
    }
  }

  _samples << std::fixed << std::setprecision(6) << simulationTimeS << ',';
  writeBoolean(_samples, snapshot.gpsValid);
  _samples << ',';
  if (snapshot.gpsValid) _samples << snapshot.gpsXM;
  _samples << ',';
  if (snapshot.gpsValid) _samples << snapshot.gpsZM;
  _samples << ',';
  writeBoolean(_samples, snapshot.compassValid);
  _samples << ',';
  if (snapshot.compassValid) _samples << snapshot.compassHeadingRad;
  _samples << ',';
  writeBoolean(_samples, snapshot.encodersValid);
  _samples << ',';
  if (snapshot.encodersValid) _samples << snapshot.leftEncoderRad;
  _samples << ',';
  if (snapshot.encodersValid) _samples << snapshot.rightEncoderRad;
  _samples << ',';
  writeBoolean(_samples, snapshot.distanceSensorsValid);
  _samples << ',';
  writeBoolean(_samples, frontImageAvailable);
  _samples << ',';
  writeBoolean(_samples, sphericalImageAvailable);
  _samples << ',';
  if (poseValid) _samples << snapshot.gpsXM - _firstPoseXM;
  _samples << ',';
  if (poseValid) _samples << snapshot.gpsZM - _firstPoseZM;
  _samples << ',';
  if (poseValid)
    _samples << std::hypot(snapshot.gpsXM - _firstPoseXM,
                           snapshot.gpsZM - _firstPoseZM);

  for (int i = 0; i < robot_config::kDistanceSensorCount; ++i) {
    _samples << ',';
    if (snapshot.distanceSensorsValid) _samples << snapshot.distanceSensorValues[i];
    _samples << ',';
    if (snapshot.distanceSensorsValid)
      _samples << snapshot.distanceMeters[i];
    _samples << ',';
    if (snapshot.distanceSensorsValid)
      writeBoolean(_samples, snapshot.distanceDetected[i]);
    _samples << ',';
    if (snapshot.distanceSensorsValid)
      writeBoolean(_samples, snapshot.distanceNearSaturated[i]);
  }
  _samples << '\n';
  _samples.flush();
  ++_sampleCount;
}

void Exp001Logger::finish(double simulationTimeS) {
  if (!isOpen() || _finished)
    return;

  _summary << "simulation_time_s=" << std::fixed << std::setprecision(3)
           << simulationTimeS << '\n'
           << "samples_written=" << _sampleCount << '\n';
  if (_hasPose) {
    _summary << std::setprecision(6)
             << "final_pose_x_m=" << _lastPoseXM << '\n'
             << "final_pose_z_m=" << _lastPoseZM << '\n'
             << "gps_displacement_m="
             << std::hypot(_lastPoseXM - _firstPoseXM,
                           _lastPoseZM - _firstPoseZM)
             << '\n';
  } else {
    _summary << "final_pose_x_m=not_measured\n"
             << "final_pose_z_m=not_measured\n"
             << "gps_displacement_m=not_measured\n";
  }
  _summary.flush();
  _samples.close();
  _summary.close();
  _finished = true;
}
