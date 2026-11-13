#ifndef RESCUE_SENSOR_SNAPSHOT_H_
#define RESCUE_SENSOR_SNAPSHOT_H_

#include <array>
#include <initializer_list>
#include <limits>

#include "../config/RobotConfig.h"

// Distancia mínima (m) de un grupo de sensores. Infinito = nada detectado.
// Lectura de un sector.
struct SectorReading {
  double distanceM = std::numeric_limits<double>::infinity();
  bool detected() const { return distanceM < std::numeric_limits<double>::infinity(); }
};

// Sectores alrededor del robot. Disposición del Pioneer2:
// lado izquierdo de delante a atrás ds0..ds7, lado derecho ds15..ds8.
//   ds0/ds15 ±10°, ds1/ds14 ±30°, ds2/ds13 ±50°, ds3/ds12 y ds4/ds11 laterales,
//   ds5..ds7 / ds8..ds10 traseros.
struct DistanceSectors {
  SectorReading front;         // ds0, ds1, ds14, ds15
  SectorReading frontLeft;     // ds0, ds1
  SectorReading frontRight;    // ds14, ds15
  SectorReading diagonalLeft;  // ds1, ds2
  SectorReading diagonalRight; // ds13, ds14
  SectorReading left;          // ds2, ds3, ds4
  SectorReading right;         // ds11, ds12, ds13
  SectorReading rearLeft;      // ds5, ds6, ds7
  SectorReading rearRight;     // ds8, ds9, ds10
};

// Lectura de todos los sensores en un mismo paso de simulación.
struct SensorSnapshot {
  double timeS = 0.0;

  bool gpsValid = false;
  double gpsXM = 0.0;
  double gpsZM = 0.0;

  bool compassValid = false;
  double compassYawRad = 0.0;  // convenio en RobotConfig.h

  bool encodersValid = false;
  double leftEncoderRad = 0.0;
  double rightEncoderRad = 0.0;

  bool distanceSensorsValid = false;
  // Respuesta de Webots ya filtrada con mediana móvil.
  std::array<double, robot_config::kDistanceSensorCount> distanceResponse{};
  // Distancia estimada (m) para obstáculos cian.
  std::array<double, robot_config::kDistanceSensorCount> distanceM{};
  std::array<bool, robot_config::kDistanceSensorCount> distanceDetected{};
  std::array<bool, robot_config::kDistanceSensorCount> distanceSaturated{};
  DistanceSectors sectors;

  // Distancia más cercana entre los sensores indicados que sí detectan algo.
  SectorReading nearest(std::initializer_list<int> indices) const {
    SectorReading reading;
    for (int index : indices)
      if (distanceDetected[index] && distanceM[index] < reading.distanceM)
        reading.distanceM = distanceM[index];
    return reading;
  }

  // Llena los sectores con los sensores que corresponden a cada uno.
  void computeSectors() {
    sectors.front = nearest({0, 1, 14, 15});
    sectors.frontLeft = nearest({0, 1});
    sectors.frontRight = nearest({14, 15});
    sectors.diagonalLeft = nearest({1, 2});
    sectors.diagonalRight = nearest({13, 14});
    sectors.left = nearest({2, 3, 4});
    sectors.right = nearest({11, 12, 13});
    sectors.rearLeft = nearest({5, 6, 7});
    sectors.rearRight = nearest({8, 9, 10});
  }
};

#endif  // RESCUE_SENSOR_SNAPSHOT_H_
