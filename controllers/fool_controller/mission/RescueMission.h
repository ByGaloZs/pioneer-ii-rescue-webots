#ifndef RESCUE_RESCUE_MISSION_H_
#define RESCUE_RESCUE_MISSION_H_

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "../common/Angles.h"
#include "../common/RunOutput.h"
#include "../config/RobotConfig.h"
#include "../control/SafetyGuard.h"
#include "../control/WheelCommand.h"
#include "../localization/Localizer.h"
#include "../mapping/OccupancyGrid.h"
#include "../mapping/SphericalMapper.h"
#include "../navigation/GoalNavigator.h"
#include "../perception/FloorSegmenter.h"
#include "../perception/VictimDetector.h"
#include "../sensors/SensorSnapshot.h"

// Misión completa del trabajo final (pasos 2, 3 y 4):
//
//   IDA        ir hasta el fondo del recinto (z >= 8.2) con mapa + A*.
//   BUSCAR     recorrer el pasillo del fondo (z ~ 8.9) hasta ver las 2 víctimas
//              (cámara frontal + esférica; VictimTracker las confirma).
//   IR_A       ir con A* hasta ~1.15 m de la víctima confirmada más cercana.
//   ACERCAR    avance directo hasta 0.85 m del centro (enunciado: < 1 m).
//   GIRO       giro completo de 360° en el sitio y mensaje en la terminal.
//   VUELTA     A* hacia la línea de salida (z <= -8.2), prefiriendo las celdas
//              por las que ya pasó (se sabe que caben).
//
// Si el tiempo no da para seguir buscando, se vuelve con las que haya
// rescatado: la vuelta tiene prioridad (límite total 240 s).
//
// Implementación solo en cabecera para no tener que tocar el Makefile.
class RescueMission {
public:
  enum class Phase { kOutbound, kSearch, kGoto, kApproach, kSpin, kReturn, kDone };

  RescueMission(bool useCamera, bool useMap, bool liveMap, bool useSphere,
                const std::string &tag)
      : _useCamera(useCamera), _useMap(useMap), _liveMap(liveMap && useMap),
        _useSphere(useSphere), _tag(tag) {}

  WheelCommand step(const SensorSnapshot &snapshot, const Localizer &localizer,
                    const FreeSpaceProfile *camera, const unsigned char *sphereImage,
                    double dtS) {
    namespace nav = robot_config::nav;
    if (_finished || !localizer.initialized()) return WheelCommand::stop();
    const Pose &pose = localizer.pose();
    if (!_started) start(snapshot, localizer);
    _timeS = snapshot.timeS;
    const double elapsed = snapshot.timeS - _startTimeS;

    trackContacts(snapshot);
    _maxZ = std::max(_maxZ, pose.zM);
    _minZ = std::min(_minZ, pose.zM);

    if (_useMap) _grid.update(snapshot, pose);
    const bool sphereTurn = _sphereStep++ % robot_config::mapping::kSphereEveryNSteps == 0;
    if (_useMap && _useSphere && sphereImage != nullptr && sphereTurn) {
      ++_sphereFrames;
      if (_sphereMapper.update(sphereImage, pose, _grid)) ++_sphereValidFrames;
    }
    detectVictims(pose, camera, sphereImage, sphereTurn);

    if (elapsed > nav::kMissionTimeLimitS) {
      finish(snapshot, localizer, false);
      return WheelCommand::stop();
    }

    WheelCommand command = WheelCommand::stop();
    const FreeSpaceProfile *navCamera = (_useCamera && camera != nullptr) ? camera : nullptr;
    switch (_phase) {
      case Phase::kOutbound:
        if (pose.zM >= nav::kOutboundDoneZ) {
          _outboundS = elapsed;
          std::cout << std::fixed << std::setprecision(1) << "[MISION] Ida completada en "
                    << elapsed << " s. Víctimas vistas hasta ahora: "
                    << _tracker.confirmedCount() << std::endl;
          printCandidates();
          enter(Phase::kSearch, pose);
          break;
        }
        command = _navigator.step(snapshot, pose, navCamera, dtS);
        break;

      case Phase::kSearch:
        command = searchStep(snapshot, pose, navCamera, elapsed, dtS);
        break;

      case Phase::kGoto:
        command = gotoStep(snapshot, pose, elapsed, dtS);
        break;

      case Phase::kApproach:
        command = approachStep(snapshot, pose, elapsed);
        break;

      case Phase::kSpin:
        command = spinStep(snapshot, pose, elapsed);
        break;

      case Phase::kReturn:
        if (pose.zM <= robot_config::mission::kReturnDoneZ) {
          finish(snapshot, localizer, true);
          return WheelCommand::stop();
        }
        command = _navigator.step(snapshot, pose, navCamera, dtS);
        break;

      case Phase::kDone:
        return WheelCommand::stop();
    }

    writeLive(snapshot, pose, elapsed);
    return _safety.filter(snapshot, command);
  }

  bool finished() const { return _finished; }

  void printStatus(const SensorSnapshot &snapshot, const Localizer &localizer) const {
    const Pose &pose = localizer.pose();
    std::cout << std::fixed << std::setprecision(1) << "[MISION] t="
              << snapshot.timeS - _startTimeS << " " << phaseName() << std::setprecision(2)
              << " pos=(" << pose.xM << ", " << pose.zM << ") yaw=" << std::setprecision(0)
              << angles::toDeg(pose.yawRad) << " victimas vistas="
              << _tracker.confirmedCount() << " rescatadas=" << _tracker.rescuedCount();
    if (_useMap)
      std::cout << " plan=" << (_navigator.planActive() ? "ok" : "NO")
                << " restante=" << std::setprecision(2) << _navigator.remainingM(pose);
    std::cout << std::endl;
  }

private:
  // ------------------------------------------------------------------ fases
  void start(const SensorSnapshot &snapshot, const Localizer &localizer) {
    namespace nav = robot_config::nav;
    const Pose &pose = localizer.pose();
    _started = true;
    _startTimeS = snapshot.timeS;
    _startX = pose.xM;
    _startZ = pose.zM;
    _startYaw = pose.yawRad;
    _startTravelledM = localizer.travelledM();
    _maxZ = _minZ = pose.zM;
    if (_useMap) _grid.update(snapshot, pose);
    _navigator.setMap(_useMap ? &_grid : nullptr);
    _navigator.setGoalLine(nav::kOutboundGoalZ, pose);
    std::cout << "[MISION] Inicio: ida hacia z=" << nav::kOutboundGoalZ << ", buscar 2 víctimas"
              << ", rescatarlas (parar < 1 m + giro 360°) y volver | cámara "
              << (_useCamera ? "SI" : "no") << " | mapa+A* " << (_useMap ? "SI" : "no")
              << " | esférica " << (_useSphere ? "SI" : "no") << std::endl;
  }

  void enter(Phase phase, const Pose &pose) {
    _phase = phase;
    _phaseStartS = _timeS;
    switch (phase) {
      case Phase::kSearch:
        _goalZ = robot_config::victims::kSearchZ;
        buildWaypoints(pose);
        _searchLeg = -1;  // se elige en searchStep
        break;
      case Phase::kReturn: {
        namespace nav = robot_config::nav;
        _goalZ = robot_config::mission::kReturnGoalZ;
        _returnStartS = _timeS - _startTimeS;
        _navigator.setPreferVisited(true);
        _navigator.setGoalLine(robot_config::mission::kReturnGoalZ, pose);
        std::cout << std::fixed << std::setprecision(1) << "[MISION] t=" << _returnStartS
                  << " s: VOLVIENDO a la salida con " << _tracker.rescuedCount()
                  << " víctima(s) rescatada(s)." << std::endl;
        (void)nav::kOutboundGoalZ;
        break;
      }
      default:
        break;
    }
  }

  void printCandidates() const {
    std::cout << std::fixed << std::setprecision(2);
    for (const VictimTracker::Candidate &c : _tracker.candidates())
      std::cout << "[MISION]   candidata (" << c.x << ", " << c.z << ") vista en " << c.frames
                << " imágenes" << (_tracker.confirmed(c) ? " (confirmada)" : "")
                << (c.rescued ? " RESCATADA" : "") << (c.failed ? " FALLIDA" : "") << std::endl;
  }

  void buildWaypoints(const Pose &pose) {
    namespace v = robot_config::victims;
    _waypoints.clear();
    const double first = std::fabs(pose.xM - v::kSearchX) < std::fabs(pose.xM + v::kSearchX)
                             ? v::kSearchX
                             : -v::kSearchX;
    _waypoints.push_back({first, v::kSearchZ});
    _waypoints.push_back({-first, v::kSearchZ});
    _waypoints.push_back({0.0, v::kSearchZ});
    _nextWaypoint = 0;
  }

  // ¿Queda tiempo para seguir con víctimas, o hay que volver ya?
  bool mustReturn(double elapsed) const {
    namespace m = robot_config::mission;
    const double reserve = m::kReturnReserveFactor * _outboundS + m::kReturnReserveFixedS;
    return elapsed + reserve > robot_config::nav::kMissionTimeLimitS - m::kReturnSafetyMarginS;
  }

  WheelCommand searchStep(const SensorSnapshot &snapshot, const Pose &pose,
                          const FreeSpaceProfile *camera, double elapsed, double dtS) {
    namespace v = robot_config::victims;
    if (_tracker.rescuedCount() >= 2 || mustReturn(elapsed)) {
      if (_tracker.rescuedCount() < 2)
        std::cout << "[MISION] Poco tiempo: se vuelve sin buscar más." << std::endl;
      enter(Phase::kReturn, pose);
      return WheelCommand::stop();
    }
    if (VictimTracker::Candidate *c = _tracker.nearestPending(pose.xM, pose.zM)) {
      _target = static_cast<int>(c - &_tracker.candidates()[0]);
      std::cout << std::fixed << std::setprecision(2) << "[MISION] Víctima confirmada en ("
                << c->x << ", " << c->z << "): voy a por ella." << std::endl;
      _navigator.setGoalPoint(c->x, c->z, v::kApproachRadiusM, pose);
      _gotoGoalX = c->x;
      _gotoGoalZ = c->z;
      _phase = Phase::kGoto;
      _phaseStartS = _timeS;
      _goalZ = c->z;
      return WheelCommand::stop();
    }
    // Barrido del pasillo del fondo.
    if (_searchLeg != _nextWaypoint) {
      if (_nextWaypoint >= static_cast<int>(_waypoints.size())) {
        std::cout << "[MISION] Búsqueda agotada: se vuelve con " << _tracker.rescuedCount()
                  << " víctima(s)." << std::endl;
        enter(Phase::kReturn, pose);
        return WheelCommand::stop();
      }
      _searchLeg = _nextWaypoint;
      _legStartS = _timeS;
      _navigator.setGoalPoint(_waypoints[_searchLeg].first, _waypoints[_searchLeg].second,
                              v::kSearchWaypointRadiusM, pose);
      std::cout << std::fixed << std::setprecision(2) << "[MISION] Buscando: tramo "
                << _searchLeg + 1 << "/" << _waypoints.size() << " hacia ("
                << _waypoints[_searchLeg].first << ", " << _waypoints[_searchLeg].second << ")"
                << std::endl;
    }
    const double dist = std::hypot(_waypoints[_searchLeg].first - pose.xM,
                                   _waypoints[_searchLeg].second - pose.zM);
    if (dist <= v::kSearchWaypointRadiusM + 0.1 || _timeS - _legStartS > v::kSearchLegTimeoutS) {
      ++_nextWaypoint;
      return WheelCommand::stop();
    }
    return _navigator.step(snapshot, pose, camera, dtS);
  }

  WheelCommand gotoStep(const SensorSnapshot &snapshot, const Pose &pose, double elapsed,
                        double dtS) {
    namespace v = robot_config::victims;
    VictimTracker::Candidate &c = _tracker.candidates()[_target];
    const double dist = std::hypot(c.x - pose.xM, c.z - pose.zM);
    if (dist <= v::kApproachRadiusM + 0.1 || _navigator.remainingM(pose) < 0.05) {
      _phase = Phase::kApproach;
      _phaseStartS = _timeS;
      return WheelCommand::stop();
    }
    if (_timeS - _phaseStartS > v::kGotoTimeoutS || mustReturn(elapsed)) {
      std::cout << "[MISION] No llego a la víctima: la dejo y sigo." << std::endl;
      c.failed = true;
      enter(Phase::kSearch, pose);
      return WheelCommand::stop();
    }
    if (std::hypot(c.x - _gotoGoalX, c.z - _gotoGoalZ) > 0.25) {
      _gotoGoalX = c.x;
      _gotoGoalZ = c.z;
      _navigator.moveGoalPoint(c.x, c.z, pose);
    }
    return _navigator.step(snapshot, pose, nullptr, dtS);  // sin cámara: la víctima es la meta
  }

  WheelCommand approachStep(const SensorSnapshot &snapshot, const Pose &pose, double elapsed) {
    namespace v = robot_config::victims;
    VictimTracker::Candidate &c = _tracker.candidates()[_target];
    const double dx = c.x - pose.xM, dz = c.z - pose.zM;
    const double dist = std::hypot(dx, dz);
    const double err = angles::wrap(std::atan2(-dx, -dz) - pose.yawRad);

    // Sin tiempo para seguir: si aún no se está a <1 m (el giro no ha podido
    // empezar) se abandona el acercamiento y se vuelve. Si ya se está a <1 m
    // se termina: el giro dura ~2.5 s y el margen de vuelta lo cubre.
    if (dist >= 1.0 && mustReturn(elapsed)) {
      std::cout << "[MISION] Poco tiempo: se abandona el acercamiento y se vuelve." << std::endl;
      enter(Phase::kReturn, pose);
      return WheelCommand::stop();
    }

    double frontIr = 1e9;
    for (int i : {0, 1, 14, 15})
      if (snapshot.distanceDetected[i]) frontIr = std::min(frontIr, snapshot.distanceM[i]);
    const bool touching = frontIr < 0.12;

    // Demasiado cerca (p. ej. la ida terminó pegada a la víctima): retroceder
    // hasta ~0.8 m antes de girar, para que el giro no la roce.
    if (dist < v::kTooCloseM && _timeS - _phaseStartS < v::kApproachTimeoutS) {
      bool rearFree = true;
      for (int i = 4; i <= 11; ++i)
        if (snapshot.distanceDetected[i] && snapshot.distanceM[i] < 0.12) rearFree = false;
      if (rearFree) return WheelCommand::of(-2.0, -2.0);
    }
    if (dist <= v::kStopDistanceM || touching ||
        _timeS - _phaseStartS > v::kApproachTimeoutS) {
      if (dist < 1.0) {
        _spinDistance = dist;
        _spinAccum = 0.0;
        _spinLastYaw = pose.yawRad;
        _phase = Phase::kSpin;
        _phaseStartS = _timeS;
        std::cout << std::fixed << std::setprecision(2) << "[VICTIMA " << _tracker.rescuedCount() + 1
                  << "] Víctima encontrada en (" << c.x << ", " << c.z << "), a " << dist
                  << " m del robot. Giro de 360°..." << std::endl;
      } else {
        std::cout << "[MISION] No consigo acercarme a menos de 1 m: abandono esta víctima."
                  << std::endl;
        c.failed = true;
        printCandidates();
        enter(Phase::kSearch, pose);
      }
      return WheelCommand::stop();
    }
    if (std::fabs(err) > 0.35)
      return WheelCommand::of(-2.0 * (err > 0 ? 1 : -1), 2.0 * (err > 0 ? 1 : -1));
    const double speed = dist > 1.05 ? 3.0 : 2.0;
    const double turn = std::max(-2.0, std::min(2.0, 4.0 * err));
    return WheelCommand::of(speed - turn, speed + turn);
  }

  WheelCommand spinStep(const SensorSnapshot &snapshot, const Pose &pose, double elapsed) {
    (void)snapshot;
    namespace v = robot_config::victims;
    _spinAccum += std::fabs(angles::wrap(pose.yawRad - _spinLastYaw));
    _spinLastYaw = pose.yawRad;
    const bool completed = _spinAccum >= v::kSpinTargetRad;
    if (completed || _timeS - _phaseStartS > v::kSpinTimeoutS) {
      VictimTracker::Candidate &c = _tracker.candidates()[_target];
      if (!completed) {
        // El giro no se completó (robot bloqueado): NO cuenta como rescate.
        std::cout << std::fixed << std::setprecision(0) << "[MISION] Giro incompleto ("
                  << angles::toDeg(_spinAccum) << "°): víctima NO rescatada." << std::endl;
        c.failed = true;
        printCandidates();
        enter(Phase::kSearch, pose);
        return WheelCommand::stop();
      }
      c.rescued = true;
      if (_useMap) _grid.markDisc(c.x, c.z, 0.3);
      _rescueTimes.push_back(elapsed);
      _rescueDistances.push_back(_spinDistance);
      std::cout << std::fixed << std::setprecision(2) << "[VICTIMA " << _tracker.rescuedCount()
                << "] Giro de 360° completo (" << std::setprecision(0) << angles::toDeg(_spinAccum)
                << "°). VICTIMA RESCATADA en (" << std::setprecision(2) << c.x << ", " << c.z
                << ") | t=" << std::setprecision(1) << elapsed << " s" << std::endl;
      enter(Phase::kSearch, pose);
      return WheelCommand::stop();
    }
    return WheelCommand::of(-v::kSpinSpeed, v::kSpinSpeed);  // giro a la izquierda
  }

  // -------------------------------------------------------------- percepción
  void detectVictims(const Pose &pose, const FreeSpaceProfile *camera,
                     const unsigned char *sphereImage, bool sphereTurn) {
    _tracker.beginFrame();
    std::vector<VictimObservation> obs;
    if (_useCamera && camera != nullptr && camera->valid)
      victim_detector::fromFront(*camera, pose, obs);
    if (_useSphere && sphereImage != nullptr && sphereTurn)
      victim_detector::fromSphere(sphereImage, pose, _sphereMapper.lastFloorBrightness(), obs);
    for (const VictimObservation &o : obs) _tracker.add(o, pose.xM, pose.zM);
    if (_phase == Phase::kOutbound || _phase == Phase::kSearch) _tracker.mergeDuplicates();
  }

  void trackContacts(const SensorSnapshot &snapshot) {
    bool near = false;
    for (int i = 0; i < robot_config::kDistanceSensorCount; ++i) {
      if (!snapshot.distanceDetected[i]) continue;
      _minIrM = std::min(_minIrM, snapshot.distanceM[i]);
      if (snapshot.distanceM[i] < 0.03 || snapshot.distanceSaturated[i]) near = true;
    }
    if (near && !_inNearContact) ++_nearContacts;
    _inNearContact = near;
  }

  // ------------------------------------------------------------------ salida
  const char *phaseName() const {
    switch (_phase) {
      case Phase::kOutbound: return "IDA";
      case Phase::kSearch: return "BUSCAR";
      case Phase::kGoto: return "IR_A_VICTIMA";
      case Phase::kApproach: return "ACERCAR";
      case Phase::kSpin: return "GIRO_360";
      case Phase::kReturn: return "VUELTA";
      case Phase::kDone: return "FIN";
    }
    return "?";
  }

  std::string stateLabel() const {
    std::string label = phaseName();
    if (_phase == Phase::kOutbound || _phase == Phase::kSearch || _phase == Phase::kGoto ||
        _phase == Phase::kReturn) {
      label += "/";
      label += _navigator.stateName();
    }
    return label;
  }

  std::string victimsLine() const {
    std::ostringstream out;
    out << "victims=";
    for (const VictimTracker::Candidate &c : _tracker.candidates())
      out << c.x << ',' << c.z << ',' << (c.rescued ? 2 : _tracker.confirmed(c) ? 1 : 0) << ' ';
    out << '\n';
    return out.str();
  }

  void writeLive(const SensorSnapshot &snapshot, const Pose &pose, double elapsed) {
    if (!_liveMap || snapshot.timeS < _nextLiveS) return;
    _nextLiveS = snapshot.timeS + robot_config::kLiveMapPeriodS;
    _grid.writeLive(run_output::path("live_map.txt"), elapsed, stateLabel(), pose, _goalZ,
                    _navigator.planner().path(), victimsLine());
  }

  void finish(const SensorSnapshot &snapshot, const Localizer &localizer, bool success) {
    _finished = true;
    const bool wasPhaseReturn = _phase == Phase::kReturn;
    _phase = Phase::kDone;
    const double elapsed = snapshot.timeS - _startTimeS;
    const double path = localizer.travelledM() - _startTravelledM;
    const int rescued = _tracker.rescuedCount();
    const double returnS = wasPhaseReturn && success ? elapsed - _returnStartS : -1.0;
    // Líneas negras (-6 ... 6) cruzadas en la vuelta, hacia la línea de salida.
    int linesBack = 0;
    for (double line = 6.0; line >= -6.0 - 1e-9; line -= 2.0)
      if (_minZ < line && _maxZ > line && localizer.pose().zM < line) ++linesBack;

    std::cout << "\n[RESULT] " << (success ? "MISION COMPLETA" : "TIEMPO AGOTADO") << std::fixed
              << std::setprecision(1) << " | tiempo " << elapsed << " s | ida "
              << _outboundS << " s | víctimas rescatadas " << rescued << "/2"
              << " | vuelta " << (returnS >= 0 ? returnS : 0.0) << " s | líneas de vuelta "
              << linesBack << " | recorrido " << std::setprecision(2) << path
              << " m | casi-contactos " << _nearContacts << " | frenos de emergencia "
              << _safety.interventions() << std::endl;

    std::ostringstream row;
    row << std::fixed << std::setprecision(2) << run_output::timestamp() << ','
        << run_output::resolveTag(_tag) << ',' << (_useCamera ? "cam" : "nocam") << ','
        << _startX << ',' << _startZ << ',' << std::setprecision(0) << angles::toDeg(_startYaw)
        << ',' << (success ? 1 : 0) << ',' << rescued << ',' << std::setprecision(1) << elapsed
        << ',' << _outboundS << ',' << (_rescueTimes.size() > 0 ? _rescueTimes[0] : -1.0) << ','
        << (_rescueTimes.size() > 1 ? _rescueTimes[1] : -1.0) << ',' << returnS << ','
        << std::setprecision(2) << (_rescueDistances.size() > 0 ? _rescueDistances[0] : -1.0)
        << ',' << (_rescueDistances.size() > 1 ? _rescueDistances[1] : -1.0) << ',' << linesBack
        << ',' << path << ',' << _nearContacts << ',' << _safety.interventions() << ','
        << std::setprecision(3) << (_minIrM < 1e8 ? _minIrM : -1.0) << ','
        << std::setprecision(0)
        << (_sphereFrames > 0 ? 100.0 * _sphereValidFrames / _sphereFrames : 0.0);
    const std::string header =
        "when,tag,camera,start_x,start_z,start_yaw_deg,success,victims,time_s,outbound_s,"
        "victim1_s,victim2_s,return_s,victim1_dist_m,victim2_dist_m,lines_back,path_m,"
        "near_contacts,emergency_stops,min_ir_m,sphere_valid_pct";
    const std::string csv = run_output::path("mission_results.csv");
    std::string firstLine;
    {
      std::ifstream in(csv);
      std::getline(in, firstLine);
    }
    if (!firstLine.empty() && firstLine != header)
      std::rename(csv.c_str(),
                  run_output::path("mission_results_old_" + std::to_string(std::time(nullptr)) +
                                   ".csv").c_str());
    const bool exists = !firstLine.empty() && firstLine == header;
    std::ofstream out(csv, std::ios::app);
    if (!exists) out << header << '\n';
    out << row.str() << '\n';
    std::cout << "[RESULT] Fila añadida a results/mission_results.csv" << std::endl;
    if (_liveMap)
      _grid.writeLive(run_output::path("live_map.txt"), elapsed, success ? "FIN_OK" : "FIN_TIEMPO",
                      localizer.pose(), _goalZ, _navigator.planner().path(), victimsLine());
    if (_useMap) {
      const std::string mapFile = "map_" + run_output::resolveTag(_tag) + "_mission.txt";
      _grid.writeAscii(run_output::path(mapFile), _navigator.planner().path());
      std::cout << "[RESULT] Mapa guardado en results/" << mapFile << std::endl;
    }
  }

  // ----------------------------------------------------------------- estado
  bool _useCamera, _useMap, _liveMap, _useSphere;
  std::string _tag;
  bool _started = false, _finished = false;
  Phase _phase = Phase::kOutbound;
  double _phaseStartS = 0.0, _timeS = 0.0, _startTimeS = 0.0;
  double _startX = 0.0, _startZ = 0.0, _startYaw = 0.0, _startTravelledM = 0.0;
  double _maxZ = -1e9, _minZ = 1e9;
  double _outboundS = 60.0, _returnStartS = 0.0;
  double _goalZ = 8.6;

  OccupancyGrid _grid;
  SphericalMapper _sphereMapper;
  GoalNavigator _navigator;
  SafetyGuard _safety;
  VictimTracker _tracker;
  int _sphereStep = 0, _sphereFrames = 0, _sphereValidFrames = 0;
  double _nextLiveS = 0.0;

  // Búsqueda y rescate.
  std::vector<std::pair<double, double>> _waypoints;
  int _nextWaypoint = 0, _searchLeg = -1;
  double _legStartS = 0.0;
  int _target = -1;
  double _gotoGoalX = 0.0, _gotoGoalZ = 0.0;
  double _spinAccum = 0.0, _spinLastYaw = 0.0, _spinDistance = 0.0;
  std::vector<double> _rescueTimes, _rescueDistances;

  // Métricas.
  double _minIrM = 1e9;
  int _nearContacts = 0;
  bool _inNearContact = false;
};

#endif  // RESCUE_RESCUE_MISSION_H_
