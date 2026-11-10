#include "Exp002Logger.h"

#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

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
  return _mkdir(path.c_str()) == 0;
#else
  return mkdir(path.c_str(), 0775) == 0;
#endif
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

std::string resultsRoot() {
  const char *configuredRoot = std::getenv("EXP002_RESULTS_ROOT");
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
      "experiments/exp_002_visual_ranging/results";
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

std::string runName() {
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

Exp002Logger::Exp002Logger()
    : _runDirectory(),
      _trialId("unlabeled"),
      _singleShot(false),
      _frames(),
      _measurements(),
      _summary(),
      _frameCount(0),
      _savedImageCount(0),
      _finished(false) {}

bool Exp002Logger::initialize(int timeStepMs,
                              const webots::Camera *sphericalCamera,
                              const std::string &trialId,
                              bool singleShot) {
  if (sphericalCamera == 0) {
    std::cerr << "EXP-002 logger: spherical camera is unavailable."
              << std::endl;
    return false;
  }

  _trialId = trialId;
  _singleShot = singleShot;
  const std::string root = resultsRoot();
  if (root.empty() || (!isDirectory(root) && !makeDirectory(root) &&
                       !isDirectory(root))) {
    std::cerr << "EXP-002 logger: results directory not found. Start Webots "
                 "from within the project or set EXP002_RESULTS_ROOT."
              << std::endl;
    return false;
  }

  const std::string baseName = runName();
  for (unsigned int suffix = 0; suffix < 1000; ++suffix) {
    std::ostringstream candidate;
    candidate << root << '/' << baseName;
    if (suffix > 0)
      candidate << '_' << suffix;
    if (makeDirectory(candidate.str())) {
      _runDirectory = candidate.str();
      break;
    }
  }
  if (_runDirectory.empty()) {
    std::cerr << "EXP-002 logger: could not create a unique run directory."
              << std::endl;
    return false;
  }

  _frames.open((_runDirectory + "/frames.csv").c_str());
  _measurements.open((_runDirectory + "/visual_measurements.csv").c_str());
  _summary.open((_runDirectory + "/summary.txt").c_str());
  if (!_frames || !_measurements || !_summary) {
    std::cerr << "EXP-002 logger: could not create run files in "
              << _runDirectory << std::endl;
    return false;
  }

  _frames << "frame_index,simulation_time_s,gps_valid,gps_x_m,gps_z_m,"
             "compass_valid,heading_rad,image_available,image_saved,image_file,"
             "image_width_px,image_height_px,camera_fov_rad,trial_id\n";
  _measurements
      << "frame_file,scenario,object_type,reference_distance_m,sector,"
         "radial_boundary_px,estimated_distance_m,absolute_error_m,usable,"
         "unknown_reason,occlusion,visibility_condition,notes\n";
  _summary << "experiment_id=EXP-002\n"
           << "run_name=" << _runDirectory.substr(_runDirectory.find_last_of("/\\") + 1) << '\n'
           << "trial_id=" << _trialId << '\n'
           << "capture_mode=" << (_singleShot ? "single_after_1s" : "periodic") << '\n'
           << "scenario=not_provided_to_controller\n"
           << "world_path=not_read_by_controller\n"
           << "webots_timestep_ms=" << timeStepMs << '\n'
           << "spherical_camera_resolution=" << sphericalCamera->getWidth()
           << 'x' << sphericalCamera->getHeight() << '\n'
           << "spherical_camera_fov_rad=" << std::fixed
           << std::setprecision(6) << sphericalCamera->getFov() << '\n'
           << "image_format=JPEG\n"
           << "capture_period_simulated_s=1\n"
           << "reference_distances=to_be_measured_offline\n"
           << "status=recording\n";
  _frames.flush();
  _measurements.flush();
  _summary.flush();
  return true;
}

bool Exp002Logger::isOpen() const {
  return _frames.is_open() && _measurements.is_open() && _summary.is_open();
}

const std::string &Exp002Logger::runDirectory() const {
  return _runDirectory;
}

bool Exp002Logger::capture(double simulationTimeS,
                           const webots::Camera *sphericalCamera,
                           const SensorSnapshot &snapshot) {
  if (!isOpen() || _finished || sphericalCamera == 0)
    return false;

  std::ostringstream imageName;
  imageName << "frame_" << std::setw(6) << std::setfill('0') << _frameCount
            << ".jpg";
  const std::string relativeImagePath = imageName.str();
  const std::string absoluteImagePath = _runDirectory + '/' + relativeImagePath;
  const bool imageAvailable = sphericalCamera->getImage() != 0;
  const bool imageSaved =
      imageAvailable && sphericalCamera->saveImage(absoluteImagePath, 100) == 0;

  _frames << _frameCount << ',' << std::fixed << std::setprecision(6)
          << simulationTimeS << ',';
  writeBoolean(_frames, snapshot.gpsValid);
  _frames << ',';
  if (snapshot.gpsValid) _frames << snapshot.gpsXM;
  _frames << ',';
  if (snapshot.gpsValid) _frames << snapshot.gpsZM;
  _frames << ',';
  writeBoolean(_frames, snapshot.compassValid);
  _frames << ',';
  if (snapshot.compassValid) _frames << snapshot.compassHeadingRad;
  _frames << ',';
  writeBoolean(_frames, imageAvailable);
  _frames << ',';
  writeBoolean(_frames, imageSaved);
  _frames << ',';
  if (imageSaved) _frames << relativeImagePath;
  _frames << ',' << sphericalCamera->getWidth() << ','
          << sphericalCamera->getHeight() << ',' << sphericalCamera->getFov()
          << ',' << _trialId << '\n';

  if (imageSaved)
    _measurements << relativeImagePath;
  for (int column = 1; column < 13; ++column)
    _measurements << ',';
  _measurements << '\n';
  _frames.flush();
  _measurements.flush();
  ++_frameCount;
  if (imageSaved)
    ++_savedImageCount;
  return imageSaved;
}

void Exp002Logger::finish(double simulationTimeS) {
  if (!isOpen() || _finished)
    return;

  _summary << "status=completed\n"
           << "capture_count=" << _frameCount << '\n'
           << "images_saved=" << _savedImageCount << '\n'
           << "simulation_time_s=" << std::fixed << std::setprecision(3)
           << simulationTimeS << '\n';
  _summary.flush();
  _frames.close();
  _measurements.close();
  _summary.close();
  _finished = true;
}
