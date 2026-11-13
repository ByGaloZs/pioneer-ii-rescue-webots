#include "GoalNavigator.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "../common/Angles.h"
#include "../config/RobotConfig.h"

namespace {

namespace nav = robot_config::nav;
constexpr double kInf = std::numeric_limits<double>::infinity();

// Ángulo de cada sensor respecto al frente del robot (rad, positivo = izquierda).
const double kSensorAngleDeg[robot_config::kDistanceSensorCount] = {
    10, 30, 50, 90, 90, 130, 150, 170, -170, -150, -130, -90, -90, -50, -30, -10};

double clamp(double v, double limit) { return std::max(-limit, std::min(limit, v)); }

// Distancia de un IR, o infinito si no detecta nada.
double ir(const SensorSnapshot &s, int index) {
  return s.distanceDetected[index] ? s.distanceM[index] : kInf;
}

// ¿Se puede retroceder? Mirar también las diagonales traseras: las esquinas
// traseras del Pioneer son lo que más sobresale.
bool rearClear(const SensorSnapshot &s, double margin) {
  return std::min(std::min(ir(s, 6), ir(s, 7)), std::min(ir(s, 8), ir(s, 9))) > margin &&
         std::min(ir(s, 5), ir(s, 10)) > 0.7 * margin;
}

// Diferencial de ruedas: u > 0 gira a la izquierda (aumenta el yaw).
WheelCommand drive(double speed, double turn) {
  return WheelCommand::of(speed - turn, speed + turn);
}

}  // namespace

// Fija una meta puntual (víctima) y reinicia el estado y las estadísticas.
void GoalNavigator::setGoalPoint(double x, double z, double radiusM, const Pose &pose) {
  _pointGoal = true;
  _gx = x;
  _gz = z;
  _gRadius = radiusM;
  _state = State::kAlign;
  _alignDir = 0;
  _lastYaw = pose.yawRad;
  _lastPlanS = -10.0;
  if (_grid != nullptr) replanIfNeeded(pose, true);
}

// Actualiza la posición de la meta puntual sin reiniciar el estado.
void GoalNavigator::moveGoalPoint(double x, double z, const Pose &pose) {
  _gx = x;
  _gz = z;
  if (_grid != nullptr) replanIfNeeded(pose, true);
}

// Fija una meta de tipo línea (z = constante).
void GoalNavigator::setGoalLine(double zTarget, const Pose &pose) {
  _pointGoal = false;
  _zTarget = zTarget;
  _direction = zTarget >= pose.zM ? 1.0 : -1.0;
  _goalYaw = _direction > 0 ? angles::kPi : 0.0;  // +Z: yaw = pi ; -Z: yaw = 0
  _state = State::kAlign;
  _lastYaw = pose.yawRad;
  _lastPlanS = -10.0;
  if (_grid != nullptr) replanIfNeeded(pose, true);
}

// Distancia que falta: longitud del plan, o línea recta si no hay plan.
double GoalNavigator::remainingM(const Pose &pose) const {
  if (planActive()) return _planner.remainingM(pose);
  if (_pointGoal) return std::max(0.0, std::hypot(_gx - pose.xM, _gz - pose.zM) - _gRadius);
  return std::max(0.0, (_zTarget - pose.zM) * _direction);
}

// Orientación de la meta en el convenio de yaw.
double GoalNavigator::currentGoalYaw(const Pose &pose) const {
  if (planActive()) return _planner.lookaheadYaw(pose);
  if (_pointGoal) return std::atan2(-(_gx - pose.xM), -(_gz - pose.zM));
  return _goalYaw;
}

// Replanifica si es forzado, si pasó el periodo o si el mapa cambió.
void GoalNavigator::replanIfNeeded(const Pose &pose, bool force) {
  namespace m = robot_config::mapping;
  if (_grid == nullptr) return;
  const double since = _timeS - _lastPlanS;
  const bool mapChanged = _grid->version() != _planVersion;
  if (!force && since < m::kReplanPeriodS && !(mapChanged && since >= m::kReplanMinPeriodS))
    return;
  if (_pointGoal)
    _planner.planPoint(*_grid, pose, _gx, _gz, _gRadius);
  else
    _planner.plan(*_grid, pose, _zTarget, _direction);
  _lastPlanS = _timeS;
  _planVersion = _grid->version();
}

const char *GoalNavigator::stateName() const {
  switch (_state) {
    case State::kAlign: return "ALIGN";
    case State::kGoToGoal: return "GO_TO_GOAL";
    case State::kFollowWall: return _wallSide > 0 ? "FOLLOW_WALL_L" : "FOLLOW_WALL_R";
    case State::kReverseOut: return "REVERSE_OUT";
    case State::kTurnAround: return "TURN_AROUND";
    case State::kRecover: return "RECOVER";
  }
  return "?";
}

// Máquina de estados principal; al final pasa el comando por el detector de atasco.
WheelCommand GoalNavigator::step(const SensorSnapshot &snapshot, const Pose &pose,
                                 const FreeSpaceProfile *camera, double dtS) {
  _timeS = snapshot.timeS;
  _yawNow = pose.yawRad;
  replanIfNeeded(pose, false);
  const double goalRel = angles::wrap(currentGoalYaw(pose) - pose.yawRad);
  _turnAccumulated += std::fabs(angles::wrap(pose.yawRad - _lastYaw));
  _lastYaw = pose.yawRad;

  WheelCommand command;
  switch (_state) {
    case State::kAlign:
      if (std::fabs(goalRel) < 0.3) {
        _state = State::kGoToGoal;
        _alignDir = 0;
        command = goToGoal(snapshot, pose, camera, goalRel);
      } else {
        if (_alignDir == 0) {
          // Sentido por defecto: el más corto. Si el giro es grande y la trasera
          // de ese lado está pegada a algo, girar al revés (la trasera barre hacia fuera).
          int direction = goalRel > 0 ? 1 : -1;
          if (std::fabs(goalRel) > 1.6) {
            const double rearLeft = std::min(ir(snapshot, 4), std::min(ir(snapshot, 5), ir(snapshot, 6)));
            const double rearRight = std::min(ir(snapshot, 11), std::min(ir(snapshot, 10), ir(snapshot, 9)));
            const double sweptRear = direction > 0 ? rearRight : rearLeft;
            const double otherRear = direction > 0 ? rearLeft : rearRight;
            if (sweptRear < 0.12 && otherRear > sweptRear + 0.03) direction = -direction;
          }
          enterAlign(direction);
        }
        // Si no consigue girar en unos segundos (encajado), probar el otro sentido.
        if (!_alignFlipped && _timeS - _alignStartS > 4.0) {
          _alignDir = -_alignDir;
          _alignFlipped = true;
          _alignStartS = _timeS;
        }
        command = safePivot(snapshot, _alignDir, nav::kPivotSpeed);
      }
      break;

    case State::kGoToGoal:
      if (std::fabs(goalRel) > nav::kAlignAngleRad) {
        _state = State::kAlign;
        _alignDir = 0;
        command = WheelCommand::stop();
      } else {
        command = goToGoal(snapshot, pose, camera, goalRel);
      }
      break;

    case State::kFollowWall:
      command = followWall(snapshot, pose, camera, goalRel, dtS);
      break;

    case State::kReverseOut:
      if (_timeS - _reverseStartS > 0.8 || !rearClear(snapshot, 0.10)) {
        _state = State::kTurnAround;
        _turnAccumulated = 0.0;
        command = WheelCommand::stop();
      } else {
        command = WheelCommand::of(-2.5, -2.5);
      }
      break;

    case State::kTurnAround:
      // Media vuelta (~150°) y después rodear por el otro lado.
      if (_turnAccumulated >= angles::toRad(150.0)) {
        _wallSide = -_wallSide;
        _switchedThisObstacle = true;
        _wallLostS = 0.0;
        _followStartS = _timeS;
        _engaging = false;
        _state = State::kFollowWall;
        command = WheelCommand::stop();
      } else {
        // Girar HACIA la pared: la parte trasera (donde el Pioneer sobresale más)
        // barre hacia fuera, lejos de ella. Si delante hay algo, girar al revés.
        const bool frontClear = std::min(ir(snapshot, 0), ir(snapshot, 15)) > 0.15 &&
                                ir(snapshot, _wallSide > 0 ? 1 : 14) > 0.12;
        command = safePivot(snapshot, frontClear ? _wallSide : -_wallSide, nav::kPivotSpeed);
      }
      break;

    case State::kRecover: {
      const double elapsed = _timeS - _recoverStartS;
      if (elapsed < 0.6) {
        command = rearClear(snapshot, 0.12) ? WheelCommand::of(-2.5, -2.5)
                                            : safePivot(snapshot, _recoverDirection, nav::kPivotSpeed);
      } else if (elapsed < 1.3) {
        command = safePivot(snapshot, _recoverDirection, nav::kPivotSpeed);
      } else {
        // Si se estaba rodeando un obstáculo, seguir rodeándolo (no volver a la
        // trampa persiguiendo la meta).
        _state = _resumeFollowing ? State::kFollowWall : State::kAlign;
        _alignDir = 0;
        _engaging = _resumeFollowing;
        command = WheelCommand::stop();
      }
      break;
    }
  }

  if (_state != State::kRecover && updateStuckDetector(pose, command, _timeS)) {
    ++_recoveries;
    _resumeFollowing = _state == State::kFollowWall || _state == State::kTurnAround;
    _state = State::kRecover;
    _recoverStartS = _timeS;
    // Girar hacia el lado con más espacio.
    const double left = std::min(ir(snapshot, 1), ir(snapshot, 2));
    const double right = std::min(ir(snapshot, 14), ir(snapshot, 13));
    _recoverDirection = left > right ? 1 : -1;
    command = rearClear(snapshot, 0.12) ? WheelCommand::of(-2.5, -2.5) : WheelCommand::stop();
  }
  return command;
}

// Avance hacia la meta: rodea si hay obstáculo; si no, escoge rumbo y velocidad con la cámara.
WheelCommand GoalNavigator::goToGoal(const SensorSnapshot &s, const Pose &pose,
                                     const FreeSpaceProfile *camera, double goalRel) {
  // 1. ¿Obstáculo delante? -> rodearlo.
  const double front = std::min(ir(s, 0), ir(s, 15));
  const double diagonal = std::min(ir(s, 1), ir(s, 14));
  // Paso más estrecho que el robot: obstáculos cerca a ambos lados a la vez.
  const double leftSide = std::min(ir(s, 2), ir(s, 3));
  const double rightSide = std::min(ir(s, 12), ir(s, 13));
  const bool tooNarrow = leftSide < 0.10 && rightSide < 0.10;
  if (front < nav::kHitFrontM || diagonal < nav::kHitDiagonalM || tooNarrow) {
    startFollowing(s, pose, goalRel);
    return WheelCommand::stop();
  }

  // 2. Dirección y velocidad.
  double desiredRel = goalRel;
  double speed = nav::kCruiseSpeedNoCamera;
  if (camera != nullptr && camera->valid && std::fabs(goalRel) < 0.45) {
    const double needed = std::min(nav::kCameraNeededClearanceM, remainingM(pose) + 0.3);
    double bestCost = kInf, bestRel = goalRel, bestClearance = 0.0;
    double widestRel = goalRel, widestClearance = -1.0;
    for (double h = -nav::kCameraSearchAngleRad; h <= nav::kCameraSearchAngleRad + 1e-9;
         h += angles::toRad(2.0)) {
      const double clearance = camera->corridorClearance(h, nav::kRobotHalfWidthM, 10.0);
      if (clearance > widestClearance + 0.05 ||
          (std::fabs(clearance - widestClearance) <= 0.05 &&
           std::fabs(h - goalRel) < std::fabs(widestRel - goalRel))) {
        widestClearance = clearance;
        widestRel = h;
      }
      if (clearance >= needed) {
        const double cost = std::fabs(h - goalRel) + 0.15 * std::fabs(h - _previousDesiredRel);
        if (cost < bestCost) {
          bestCost = cost;
          bestRel = h;
          bestClearance = clearance;
        }
      }
    }
    if (bestCost == kInf) {  // ningún hueco suficiente: ir hacia el más amplio
      bestRel = widestRel;
      bestClearance = widestClearance;
    }
    if (bestClearance >= 0.9) {
      desiredRel = bestRel;
      if (std::fabs(bestRel - goalRel) > angles::toRad(4.0)) {
        _cameraPreferredSide = bestRel > goalRel ? 1 : -1;
        _cameraPreferenceTimeS = _timeS;
      }
    }
    _previousDesiredRel = desiredRel;
    _lastClearance = bestClearance;
    const double t = (bestClearance - nav::kCameraSlowClearanceM) /
                     (nav::kCameraFastClearanceM - nav::kCameraSlowClearanceM);
    speed = nav::kCameraMinSpeed +
            (nav::kCameraMaxSpeed - nav::kCameraMinSpeed) * std::max(0.0, std::min(1.0, t));
  }
  // Con cualquier IR delantero detectando, ir despacio pase lo que pase.
  for (int i : {0, 1, 2, 13, 14, 15})
    if (s.distanceDetected[i]) speed = std::min(speed, nav::kNearObstacleSpeed);

  // 3. Repulsión lateral para no rozar al pasar junto a algo.
  double repulsion = 0.0;
  for (int i : {2, 3}) {
    const double d = ir(s, i);
    if (d < nav::kSideRepulsionM) repulsion -= (nav::kSideRepulsionM - d) * nav::kSideRepulsionGain;
  }
  for (int i : {12, 13}) {
    const double d = ir(s, i);
    if (d < nav::kSideRepulsionM) repulsion += (nav::kSideRepulsionM - d) * nav::kSideRepulsionGain;
  }

  const double turn = clamp(nav::kHeadingGain * angles::wrap(desiredRel), nav::kMaxHeadingCorrection) +
                      repulsion;
  return drive(speed, turn);
}

// Prepara el rodeo del obstáculo.
void GoalNavigator::startFollowing(const SensorSnapshot &s, const Pose &pose, double goalRel) {
  ++_encounters;
  // Con mapa: replanificar ya (el obstáculo acaba de entrar en el mapa) y usar
  // el lado por el que el plan rodea el obstáculo.
  double planRel = 0.0;
  if (_grid != nullptr) {
    replanIfNeeded(pose, true);
    if (planActive()) planRel = angles::wrap(currentGoalYaw(pose) - pose.yawRad);
  }
  const double leftNear = std::min(ir(s, 0), std::min(ir(s, 1), ir(s, 2)));
  const double rightNear = std::min(ir(s, 15), std::min(ir(s, 14), ir(s, 13)));

  // ¿Pasar por la izquierda del obstáculo?
  bool passLeft;
  if (std::fabs(planRel) > 0.1) {
    passLeft = planRel > 0.0;             // el plan ya sabe por dónde se rodea
  } else if (_cameraPreferredSide != 0 && _timeS - _cameraPreferenceTimeS < 2.0) {
    passLeft = _cameraPreferredSide > 0;  // la cámara vio el hueco antes de llegar
  } else if (leftNear < rightNear - 0.03) {
    passLeft = false;                     // el obstáculo está más a la izquierda
  } else if (rightNear < leftNear - 0.03) {
    passLeft = true;
  } else if (std::fabs(pose.xM) > 2.5) {
    // Cerca de una pared lateral del recinto: ir hacia el centro (x = 0).
    const double leftYaw = pose.yawRad + 0.5 * angles::kPi;
    const double leftX = -std::sin(leftYaw);
    passLeft = leftX * (-pose.xM) > 0.0;
  } else {
    passLeft = goalRel > 0.0;
  }
  _wallSide = passLeft ? -1 : 1;  // pasar por la izquierda deja la pared a la derecha
  _hitRemaining = remainingM(pose);
  _followStartS = _timeS;
  _wallLostS = 0.0;
  _switchedThisObstacle = false;
  _planBackwardS = 0.0;
  _engaging = true;
  _state = State::kFollowWall;
}

// Revisa con IR y cámara si se puede avanzar hacia la meta.
bool GoalNavigator::goalDirectionFree(const SensorSnapshot &s, const FreeSpaceProfile *camera,
                                      double goalRel) const {
  for (int i = 0; i < robot_config::kDistanceSensorCount; ++i) {
    const double diff = angles::wrap(angles::toRad(kSensorAngleDeg[i]) - goalRel);
    if (std::fabs(diff) < angles::toRad(45.0) && s.distanceDetected[i]) return false;
  }
  if (camera != nullptr && camera->valid && std::fabs(goalRel) < 0.35 &&
      camera->corridorClearance(goalRel, nav::kRobotHalfWidthM, 10.0) < 0.7)
    return false;
  return true;
}

// Rodea la pared: mantiene distancia y paralelismo, y decide cuándo soltarla.
WheelCommand GoalNavigator::followWall(const SensorSnapshot &s, const Pose &pose,
                                       const FreeSpaceProfile *camera, double goalRel,
                                       double dtS) {
  const int side = _wallSide;
  const double lateralFront = ir(s, side > 0 ? 3 : 12);
  const double lateralRear = ir(s, side > 0 ? 4 : 11);
  const double diagonal50 = ir(s, side > 0 ? 2 : 13);
  const double diagonal30 = ir(s, side > 0 ? 1 : 14);
  const double otherDiagonal30 = ir(s, side > 0 ? 14 : 1);
  const double front = std::min(ir(s, 0), ir(s, 15));
  const double remaining = remainingM(pose);

  // 0. Al empezar a rodear: girar en el sitio hasta dejar el obstáculo de lado.
  if (_engaging) {
    // Terminar cuando ni el frente ni las diagonales del lado de la pared lo ven
    // cerca: el robot queda paralelo o ligeramente separado del obstáculo.
    if (front == kInf && diagonal30 == kInf && otherDiagonal30 > nav::kHitDiagonalM &&
        diagonal50 > 0.17)
      _engaging = false;
    else
      return safePivot(s, -side, nav::kPivotSpeed);
  }

  // 1. Esquina interior u obstáculo delante: girar alejándose de la pared. Si la
  //    pared lateral está muy cerca, retroceder antes: al girar en el sitio la
  //    parte trasera del Pioneer (0.26 m detrás del eje) barre hacia la pared.
  if (front < nav::kWallFrontBlockM || diagonal30 < 0.12 || otherDiagonal30 < 0.10) {
    _wallLostS = 0.0;
    const double lateralNow = std::min(lateralFront, lateralRear);
    const bool rearFree = rearClear(s, 0.15);
    if (lateralNow < 0.12 && rearFree && front < 0.10) return WheelCommand::of(-2.0, -2.0);
    return safePivot(s, -side, nav::kPivotSpeed);
  }

  // 1b. Cuña: el paso se estrecha delante por el lado contrario a la pared.
  //     Salir marcha atrás y rodear por el otro lado antes de quedar encajado.
  const double otherDiagonal50 = ir(s, side > 0 ? 13 : 2);
  const bool wallAlongside = lateralFront < 0.19 || lateralRear < 0.19;
  if (!_switchedThisObstacle && wallAlongside && otherDiagonal30 < 0.18 &&
      otherDiagonal50 < 0.18) {
    ++_sideSwitches;
    _switchedThisObstacle = true;
    _state = State::kReverseOut;
    _reverseStartS = _timeS;
    return WheelCommand::stop();
  }

  // 2. ¿Se puede dejar la pared? Avance real hacia la meta y dirección libre.
  // No soltar la pared si la meta queda hacia el lado de la pared y esta aún se ve
  // (evita el zigzag "suelto-choco-suelto" contra paredes en diagonal).
  const bool wallStillSeen = lateralFront < 0.19 || lateralRear < 0.19 || diagonal50 < 0.19;
  const bool goalNotBehindWall =
      side * goalRel <= angles::toRad(45.0) && !(side * goalRel > angles::toRad(15.0) && wallStillSeen);
  namespace m = robot_config::mapping;
  // Con plan, la dirección del plan ya rodea lo que hay en el mapa: basta con
  // llevar un rato rodeando para poder soltar la pared.
  const bool progressed = remaining < _hitRemaining - nav::kLeaveProgressM ||
                          (planActive() && _timeS - _followStartS > m::kPlanLeaveAfterS);
  if (progressed && goalNotBehindWall && goalDirectionFree(s, camera, goalRel)) {
    _state = std::fabs(goalRel) > nav::kAlignAngleRad ? State::kAlign : State::kGoToGoal;
    _alignDir = 0;
    return WheelCommand::stop();
  }

  // 2b. Con plan: si el plan pide volver hacia atrás de forma sostenida (por
  //     aquí es un callejón o ya se recorrió), dejar la pared y seguir el plan.
  if (planActive()) {
    _planBackwardS = std::fabs(goalRel) > m::kPlanBackwardRad ? _planBackwardS + dtS : 0.0;
    if (_planBackwardS > m::kPlanBackwardTimeS) {
      ++_sideSwitches;
      _planBackwardS = 0.0;
      // Girar HACIA la pared (la trasera barre hacia fuera), salvo si delante hay algo.
      const bool frontClear = std::min(ir(s, 0), ir(s, 15)) > 0.15 &&
                              ir(s, _wallSide > 0 ? 1 : 14) > 0.12;
      enterAlign(frontClear ? _wallSide : -_wallSide);
      return WheelCommand::stop();
    }
  }

  // 3. Rodeando se aleja demasiado de la meta (o tarda mucho): rodear por el otro lado.
  //    Con plan, la regresión la decide el plan (2b); queda el tiempo máximo.
  if (!_switchedThisObstacle &&
      ((!planActive() && remaining > _hitRemaining + nav::kRegressionLimitM) ||
       _timeS - _followStartS > nav::kFollowTimeoutS)) {
    ++_sideSwitches;
    _state = State::kTurnAround;
    _turnAccumulated = 0.0;
    return WheelCommand::stop();
  }

  // 4. Pared perdida (esquina exterior): seguir un poco recto y luego girar hacia ella.
  const bool wallSeen = lateralFront < kInf || lateralRear < kInf || diagonal50 < kInf;
  if (!wallSeen) {
    _wallLostS += dtS;
    if (_wallLostS < 0.4) return drive(3.0, 0.0);
    return drive(2.5, side * 1.1);  // arco alrededor de la esquina
  }
  _wallLostS = 0.0;

  // 5. Mantener la distancia a la pared y el paralelismo.
  double lateral = std::min(lateralFront, lateralRear);
  if (diagonal50 < kInf) lateral = std::min(lateral, diagonal50 * 0.77);
  double align = 0.0;  // > 0: el morro se separa de la pared
  if (lateralFront < kInf && lateralRear < kInf) align = lateralFront - lateralRear;
  const double towardWall = clamp(nav::kWallDistanceGain * (lateral - nav::kWallTargetM) +
                                      nav::kWallAlignGain * align,
                                  nav::kMaxHeadingCorrection);
  // Muy pegado a la pared: avanzar más despacio mientras se corrige.
  const double speed = lateral < 0.09 ? 0.6 * nav::kWallFollowSpeed : nav::kWallFollowSpeed;
  return drive(speed, side * towardWall);
}

// Entra al estado ALIGN con el sentido de giro indicado.
void GoalNavigator::enterAlign(int direction) {
  _state = State::kAlign;
  _alignDir = direction;
  _alignStartS = _timeS;
  _alignFlipped = false;
}

bool GoalNavigator::updateStuckDetector(const Pose &pose, const WheelCommand &command,
                                        double timeS) {
  _windowForward += 0.5 * (command.leftRadPerSec + command.rightRadPerSec);
  _windowTurn += 0.5 * std::fabs(command.rightRadPerSec - command.leftRadPerSec);
  _windowYawChange += std::fabs(angles::wrap(pose.yawRad - _windowLastYaw));
  _windowLastYaw = pose.yawRad;
  ++_windowSteps;
  if (timeS - _windowStartS < nav::kStuckWindowS) return false;
  const double steps = std::max(1, _windowSteps);
  const double moved = std::hypot(pose.xM - _windowX, pose.zM - _windowZ);
  // Atasco: se ordena avanzar y no se avanza, o se ordena girar y no se gira.
  const bool stuckForward = _windowForward / steps > 1.5 && moved < nav::kStuckMinMoveM;
  const bool stuckTurning = _windowTurn / steps > 1.5 && _windowYawChange < 0.05;
  _windowStartS = timeS;
  _windowX = pose.xM;
  _windowZ = pose.zM;
  _windowForward = 0.0;
  _windowTurn = 0.0;
  _windowYawChange = 0.0;
  _windowSteps = 0;
  return stuckForward || stuckTurning;
}

// Giro en el sitio sin golpear con la parte trasera. El cilindro de colisión del
// Pioneer2 está 4.5 cm detrás del eje: al girar a la izquierda la trasera barre
// hacia la derecha (y al revés). Si ese lado trasero está pegado a algo, se hace
// una maniobra de escape (retroceder o avanzar) y se mantiene 0.6 s para
// no oscilar entre decisiones contrarias en pasos consecutivos.
WheelCommand GoalNavigator::safePivot(const SensorSnapshot &s, int direction, double speed) {
  if (_timeS < _escapeUntilS) return _escapeCommand;
  auto tight = [&](std::initializer_list<int> indices) {
    for (int i : indices)
      if (s.distanceDetected[i] && s.distanceM[i] < 0.09) return true;
    return false;
  };
  const bool rearTight = direction > 0 ? tight({9, 10, 11}) : tight({4, 5, 6});
  if (!rearTight) return drive(0.0, direction * speed);
  // Escapes repetidos sin que cambie la orientación (encajado en diagonal en una
  // esquina: girar choca detrás, retroceder libera el frente y se vuelve a
  // avanzar...). Tras 3, forzar un giro lento durante 1 s.
  if (_timeS - _escapeWindowStartS > 5.0 || std::fabs(angles::wrap(_yawNow - _escapeYaw)) > 0.2) {
    _escapeCount = 0;
    _escapeWindowStartS = _timeS;
    _escapeYaw = _yawNow;
  }
  if (++_escapeCount > 3) {
    _escapeCount = 0;
    _escapeCommand = drive(0.0, direction * 1.2);
    _escapeUntilS = _timeS + 1.0;
    return _escapeCommand;
  }
  if (rearClear(s, 0.12))
    _escapeCommand = WheelCommand::of(-2.0, -2.0);      // detrás hay sitio: retroceder
  else if (ir(s, 0) > 0.12 && ir(s, 15) > 0.12)
    _escapeCommand = WheelCommand::of(2.0, 2.0);        // delante hay sitio: avanzar
  else
    _escapeCommand = drive(0.0, direction * 0.8);       // sin opción: girar despacio
  _escapeUntilS = _timeS + 0.6;
  return _escapeCommand;
}
