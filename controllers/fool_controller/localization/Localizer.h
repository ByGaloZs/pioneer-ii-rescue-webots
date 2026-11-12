#ifndef RESCUE_LOCALIZER_H_
#define RESCUE_LOCALIZER_H_

#include "../sensors/SensorSnapshot.h"

// Pose en el plano del suelo (x, z) y orientación (convenio en RobotConfig.h).
struct Pose {
  double xM = 0.0;
  double zM = 0.0;
  double yawRad = 0.0;
};

// Estima la pose del robot:
//  - Orientación: brújula.
//  - Posición: GPS (si kUseGpsForPosition y es válido); si no, odometría.
// Además mantiene, solo para diagnóstico, una odometría pura (encoders +
// brújula) y una orientación calculada solo con encoders, para compararlas.
class Localizer {
public:
  // Fija el origen con la primera lectura válida. Devuelve false si aún no hay
  // GPS y brújula válidos.
  bool initialize(const SensorSnapshot &snapshot);
  void update(const SensorSnapshot &snapshot);

  bool initialized() const { return _initialized; }
  const Pose &pose() const { return _pose; }
  const Pose &odometryPose() const { return _odometry; }
  double encoderYawRad() const { return _encoderYaw; }

  // Distancia total recorrida según los encoders (m).
  double travelledM() const { return _travelledM; }
  // Giro acumulado según la brújula, sin envolver (rad). Sirve para el giro de 360°.
  double accumulatedYawRad() const { return _accumulatedYaw; }

private:
  bool _initialized = false;
  Pose _pose;
  Pose _odometry;
  double _encoderYaw = 0.0;
  double _travelledM = 0.0;
  double _accumulatedYaw = 0.0;
  double _lastLeftRad = 0.0;
  double _lastRightRad = 0.0;
  double _lastYaw = 0.0;
};

#endif  // RESCUE_LOCALIZER_H_
