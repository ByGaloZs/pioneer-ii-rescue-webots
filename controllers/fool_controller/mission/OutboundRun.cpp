#include "OutboundRun.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "../common/Angles.h"
#include "../common/RunOutput.h"
#include "../config/RobotConfig.h"

namespace nav = robot_config::nav;

namespace {
constexpr double kNearContactM = 0.03;  // IR casi en contacto: posible roce
}

OutboundRun::OutboundRun(bool useCamera, bool useMap, bool liveMap, const std::string &tag)
    : _useCamera(useCamera), _useMap(useMap), _liveMap(liveMap && useMap), _tag(tag) {}

WheelCommand OutboundRun::step(const SensorSnapshot &snapshot, const Localizer &localizer,
                               const FreeSpaceProfile *camera, double dtS) {
  if (_finished || !localizer.initialized()) return WheelCommand::stop();
  const Pose &pose = localizer.pose();
  if (!_started) {
    _started = true;
    _startTimeS = snapshot.timeS;
    _startX = pose.xM;
    _startZ = pose.zM;
    _startYaw = pose.yawRad;
    _startTravelledM = localizer.travelledM();
    if (_useMap) _grid.update(snapshot, pose);
    _navigator.setMap(_useMap ? &_grid : nullptr);
    _navigator.setGoalLine(nav::kOutboundGoalZ, pose);
    std::cout << "[RUN] Ida hacia z=" << nav::kOutboundGoalZ << " | cámara "
              << (_useCamera ? "ACTIVADA" : "desactivada") << " | mapa+A* "
              << (_useMap ? "ACTIVADO" : "desactivado") << std::endl;
  }

  // Métricas de proximidad (posibles roces).
  bool near = false;
  for (int i = 0; i < robot_config::kDistanceSensorCount; ++i) {
    if (!snapshot.distanceDetected[i]) continue;
    _minIrM = std::min(_minIrM, snapshot.distanceM[i]);
    if (snapshot.distanceM[i] < kNearContactM || snapshot.distanceSaturated[i]) near = true;
  }
  if (near && !_inNearContact) ++_nearContacts;
  _inNearContact = near;
  _maxProgressZ = std::max(_maxProgressZ, pose.zM);

  const double elapsed = snapshot.timeS - _startTimeS;
  if (pose.zM >= nav::kOutboundDoneZ) {
    finish(snapshot, localizer, true);
    return WheelCommand::stop();
  }
  if (elapsed > nav::kMissionTimeLimitS) {
    finish(snapshot, localizer, false);
    return WheelCommand::stop();
  }

  if (_useMap) _grid.update(snapshot, pose);
  if (_useCamera && camera != nullptr) {
    ++_cameraFrames;
    if (camera->valid) ++_cameraValidFrames;
    _lastFloorBrightness = camera->floorBrightness;
  }
  const WheelCommand command =
      _navigator.step(snapshot, pose, _useCamera ? camera : nullptr, dtS);
  if (_liveMap && snapshot.timeS >= _nextLiveS) {
    _nextLiveS = snapshot.timeS + robot_config::kLiveMapPeriodS;
    _grid.writeLive(run_output::path("live_map.txt"), snapshot.timeS - _startTimeS,
                    _navigator.stateName(), pose, nav::kOutboundGoalZ,
                    _navigator.planner().path());
  }
  return _safety.filter(snapshot, command);
}

void OutboundRun::printStatus(const SensorSnapshot &snapshot, const Localizer &localizer) const {
  const Pose &pose = localizer.pose();
  std::cout << std::fixed << std::setprecision(1) << "[NAV] t=" << snapshot.timeS - _startTimeS
            << " " << _navigator.stateName() << std::setprecision(2) << " pos=(" << pose.xM
            << ", " << pose.zM << ") yaw=" << std::setprecision(0) << angles::toDeg(pose.yawRad)
            << " restante=" << std::setprecision(2) << _navigator.remainingM(pose);
  if (_useCamera)
    std::cout << " libre_cam=" << _navigator.lastCameraClearanceM()
              << " luz=" << _lastFloorBrightness;
  if (_useMap)
    std::cout << " plan=" << (_navigator.planActive() ? "ok" : "NO")
              << " ocupadas=" << _grid.occupiedCount();
  std::cout << " F=";
  if (snapshot.sectors.front.detected())
    std::cout << snapshot.sectors.front.distanceM;
  else
    std::cout << "--";
  std::cout << std::endl;
}

void OutboundRun::finish(const SensorSnapshot &snapshot, const Localizer &localizer,
                         bool success) {
  _finished = true;
  const double elapsed = snapshot.timeS - _startTimeS;
  // Líneas negras (z = -6 ... 6) y amarilla final (z = 8) cruzadas.
  int lines = 0;
  for (double line = -6.0; line <= 8.0 + 1e-9; line += 2.0)
    if (_maxProgressZ > line && _startZ < line) ++lines;
  const double path = localizer.travelledM() - _startTravelledM;

  std::ostringstream row;
  row << std::fixed << std::setprecision(2) << run_output::timestamp() << ','
      << run_output::resolveTag(_tag) << ','
      << (_useCamera ? "cam" : "nocam") << ',' << _startX << ',' << _startZ << ','
      << std::setprecision(0) << angles::toDeg(_startYaw) << ',' << (success ? 1 : 0) << ','
      << std::setprecision(1) << elapsed << ',' << std::setprecision(2) << _maxProgressZ << ','
      << lines << ',' << path << ',' << _navigator.obstacleEncounters() << ','
      << _navigator.sideSwitches() << ',' << _navigator.recoveries() << ','
      << _safety.interventions() << ',' << _nearContacts << ',' << std::setprecision(3)
      << (_minIrM < 1e8 ? _minIrM : -1.0) << ',' << (_useMap ? "map" : "nomap") << ','
      << std::setprecision(0)
      << (_cameraFrames > 0 ? 100.0 * _cameraValidFrames / _cameraFrames : 0.0) << ','
      << _navigator.planner().plans();

  std::cout << "\n[RESULT] " << (success ? "IDA COMPLETADA" : "TIEMPO AGOTADO") << " | "
            << (_useCamera ? "con cámara" : "sin cámara") << std::fixed << std::setprecision(1)
            << " | tiempo " << elapsed << " s | líneas cruzadas " << lines << "/8"
            << " | recorrido " << std::setprecision(2) << path << " m | obstáculos rodeados "
            << _navigator.obstacleEncounters() << " | cambios de lado "
            << _navigator.sideSwitches() << " | atascos " << _navigator.recoveries()
            << " | frenos de emergencia " << _safety.interventions() << " | casi-contactos "
            << _nearContacts << std::endl;

  const std::string header =
      "when,tag,camera,start_x,start_z,start_yaw_deg,success,time_s,max_z,lines,path_m,"
      "obstacles,side_switches,stuck_recoveries,emergency_stops,near_contacts,min_ir_m,"
      "map,cam_valid_pct,replans";
  const std::string path_csv = run_output::path("outbound_results.csv");
  std::string firstLine;
  {
    std::ifstream in(path_csv);
    std::getline(in, firstLine);
  }
  // Si el archivo es de una versión anterior (otras columnas), apartarlo.
  if (!firstLine.empty() && firstLine != header)
    std::rename(path_csv.c_str(), run_output::path("outbound_results_old.csv").c_str());
  const bool exists = !firstLine.empty() && firstLine == header;
  std::ofstream csv(path_csv, std::ios::app);
  if (!exists) csv << header << '\n';
  csv << row.str() << '\n';
  std::cout << "[RESULT] Fila añadida a results/outbound_results.csv" << std::endl;
  if (_liveMap)
    _grid.writeLive(run_output::path("live_map.txt"), elapsed,
                    success ? "FIN_OK" : "FIN_TIEMPO", localizer.pose(), nav::kOutboundGoalZ,
                    _navigator.planner().path());
  if (_useMap) {
    const std::string mapFile = "map_" + run_output::resolveTag(_tag) + "_" +
                                (_useCamera ? "cam" : "nocam") + ".txt";
    _grid.writeAscii(run_output::path(mapFile), _navigator.planner().path());
    std::cout << "[RESULT] Mapa guardado en results/" << mapFile << std::endl;
  }
}
