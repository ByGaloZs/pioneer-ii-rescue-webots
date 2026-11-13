#ifndef RESCUE_SAFETY_GUARD_H_
#define RESCUE_SAFETY_GUARD_H_

#include "../config/RobotConfig.h"
#include "../sensors/SensorSnapshot.h"
#include "WheelCommand.h"

// Última palabra antes de los motores: si un IR delantero ve algo casi encima,
// se anula la componente de avance (el robot puede seguir girando o retroceder).
class SafetyGuard {
public:
  WheelCommand filter(const SensorSnapshot &snapshot, const WheelCommand &command) {
    // Revisa los cuatro sensores frontales (ds0, ds1, ds14, ds15).
    bool danger = false;
    for (int i : {0, 1, 14, 15})
      if (snapshot.distanceDetected[i] &&
          snapshot.distanceM[i] < robot_config::nav::kSafetyStopM)
        danger = true;
    // Componente de avance = promedio de las dos ruedas.
    const double forward = 0.5 * (command.leftRadPerSec + command.rightRadPerSec);
    if (!danger || forward <= 0.0) {
      _active = false;
      return command;
    }
    if (!_active) ++_interventions;  // contar episodios, no pasos
    _active = true;
    // Se resta el avance a ambas ruedas: queda solo el giro (o la reversa).
    return WheelCommand::of(command.leftRadPerSec - forward, command.rightRadPerSec - forward);
  }

  // Número de veces que el guardia frenó el avance.
  int interventions() const { return _interventions; }

private:
  // Indica si ya hay una intervención en curso.
  bool _active = false;
  int _interventions = 0;
};

#endif  // RESCUE_SAFETY_GUARD_H_
