#ifndef RESCUE_GOAL_NAVIGATOR_H_
#define RESCUE_GOAL_NAVIGATOR_H_

#include "../control/WheelCommand.h"
#include "../localization/Localizer.h"
#include "../mapping/OccupancyGrid.h"
#include "../perception/FloorSegmenter.h"
#include "../planning/PathPlanner.h"
#include "../sensors/SensorSnapshot.h"

// Lleva el robot hasta una línea objetivo (z = constante) en un entorno
// desconocido. Comportamientos:
//  - ALIGN:       girar en el sitio hacia la meta si está muy desviada.
//  - GO_TO_GOAL:  avanzar hacia la meta. Con cámara, elige con antelación el
//                 hueco libre más cercano a la dirección de la meta y adapta la
//                 velocidad al espacio libre que ve delante.
//  - FOLLOW_WALL: rodear el obstáculo con los infrarrojos (tipo Bug). Se deja
//                 cuando se ha avanzado hacia la meta más que en el punto de
//                 choque y la dirección de la meta está libre.
//  - TURN_AROUND: si rodeando se retrocede demasiado, da media vuelta y rodea
//                 por el otro lado (una vez por obstáculo).
//  - REVERSE_OUT: si al rodear el paso se estrecha delante (cuña), sale marcha
//                 atrás antes de dar la media vuelta, para no encajarse.
//  - RECOVER:     si el robot no avanza (atascado), retrocede y gira.
//
// Con mapa (setMap): la "dirección de la meta" ya no es +z en línea recta sino
// la del plan A* sobre el mapa de ocupación, y el progreso se mide como
// longitud del plan. Así el robot recuerda paredes y callejones ya vistos:
// si rodeando el plan pide volver atrás, deja la pared y sigue el plan.
class GoalNavigator {
public:
  enum class State { kAlign, kGoToGoal, kFollowWall, kReverseOut, kTurnAround, kRecover };

  // Meta: la línea z = zTarget. La dirección se deduce de la posición actual.
  void setGoalLine(double zTarget, const Pose &pose);
  // Mapa construido fuera (misión). nullptr: Bug puro sin memoria.
  void setMap(const OccupancyGrid *grid) { _grid = grid; }

  // camera puede ser nullptr (navegación solo con infrarrojos).
  WheelCommand step(const SensorSnapshot &snapshot, const Pose &pose,
                    const FreeSpaceProfile *camera, double dtS);

  State state() const { return _state; }
  const char *stateName() const;
  double remainingM(const Pose &pose) const;  // distancia a la línea objetivo

  // Estadísticas para el informe.
  int obstacleEncounters() const { return _encounters; }
  int sideSwitches() const { return _sideSwitches; }
  int recoveries() const { return _recoveries; }
  double lastCameraClearanceM() const { return _lastClearance; }
  bool planActive() const { return _grid != nullptr && _planner.valid(); }
  const PathPlanner &planner() const { return _planner; }

private:
  WheelCommand goToGoal(const SensorSnapshot &s, const Pose &pose,
                        const FreeSpaceProfile *camera, double goalRel);
  WheelCommand followWall(const SensorSnapshot &s, const Pose &pose,
                          const FreeSpaceProfile *camera, double goalRel, double dtS);
  void startFollowing(const SensorSnapshot &s, const Pose &pose, double goalRel);
  bool goalDirectionFree(const SensorSnapshot &s, const FreeSpaceProfile *camera,
                         double goalRel) const;
  bool updateStuckDetector(const Pose &pose, const WheelCommand &command, double timeS);
  WheelCommand safePivot(const SensorSnapshot &s, int direction, double speed);
  void replanIfNeeded(const Pose &pose, bool force);
  double currentGoalYaw(const Pose &pose) const;

  // Mapa y plan.
  const OccupancyGrid *_grid = nullptr;
  PathPlanner _planner;
  double _lastPlanS = -10.0;
  int _planVersion = -1;
  double _planBackwardS = 0.0;
  // Giro en ALIGN: sentido fijado al entrar (no cambia aunque el plan oscile).
  int _alignDir = 0;
  double _alignStartS = 0.0;
  bool _alignFlipped = false;
  void enterAlign(int direction);

  State _state = State::kAlign;
  double _zTarget = 0.0;
  double _goalYaw = 0.0;
  double _direction = 1.0;  // +1 hacia +z, -1 hacia -z

  // Seguimiento de pared.
  int _wallSide = 1;              // +1 pared a la izquierda, -1 a la derecha
  double _hitRemaining = 0.0;
  double _followStartS = 0.0;
  double _wallLostS = 0.0;
  bool _switchedThisObstacle = false;
  bool _engaging = false;          // girando para dejar el obstáculo de lado
  double _reverseStartS = 0.0;
  double _turnAccumulated = 0.0;
  double _lastYaw = 0.0;

  // Preferencia de lado aportada por la cámara.
  int _cameraPreferredSide = 0;   // +1 pasar por la izquierda, -1 por la derecha
  double _cameraPreferenceTimeS = -10.0;
  double _previousDesiredRel = 0.0;
  double _lastClearance = 0.0;

  // Detector de atasco.
  double _windowStartS = 0.0;
  double _windowX = 0.0, _windowZ = 0.0;
  double _windowForward = 0.0;
  double _windowTurn = 0.0;
  double _windowYawChange = 0.0;
  double _windowLastYaw = 0.0;
  bool _resumeFollowing = false;
  double _escapeUntilS = -1.0;
  int _escapeCount = 0;            // escapes seguidos sin conseguir girar
  double _escapeWindowStartS = -10.0;
  double _escapeYaw = 0.0;
  double _yawNow = 0.0;
  WheelCommand _escapeCommand;
  int _windowSteps = 0;
  double _recoverStartS = 0.0;
  int _recoverDirection = 1;

  double _timeS = 0.0;
  int _encounters = 0;
  int _sideSwitches = 0;
  int _recoveries = 0;
};

#endif  // RESCUE_GOAL_NAVIGATOR_H_
