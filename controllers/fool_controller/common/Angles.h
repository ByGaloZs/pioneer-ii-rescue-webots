#ifndef RESCUE_ANGLES_H_
#define RESCUE_ANGLES_H_

#include <cmath>

// Utilidades de ángulos: constantes y conversiones usadas en todo el controlador.
namespace angles {

constexpr double kPi = 3.14159265358979323846;

// Normaliza un ángulo a (-pi, pi] sumando o restando vueltas completas.
inline double wrap(double angleRad) {
  while (angleRad > kPi) angleRad -= 2.0 * kPi;
  while (angleRad <= -kPi) angleRad += 2.0 * kPi;
  return angleRad;
}

// Conversiones entre radianes y grados.
inline double toDeg(double angleRad) { return angleRad * 180.0 / kPi; }
inline double toRad(double angleDeg) { return angleDeg * kPi / 180.0; }

// Yaw (convenio de RobotConfig.h) que apunta del punto (x,z) al (tx,tz).
inline double yawTowards(double x, double z, double tx, double tz) {
  return std::atan2(-(tx - x), -(tz - z));
}

}  // namespace angles

#endif  // RESCUE_ANGLES_H_
