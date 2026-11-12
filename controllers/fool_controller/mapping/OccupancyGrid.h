#ifndef RESCUE_OCCUPANCY_GRID_H_
#define RESCUE_OCCUPANCY_GRID_H_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

#include "../config/RobotConfig.h"
#include "../localization/Localizer.h"
#include "../sensors/SensorSnapshot.h"

// Mapa de ocupación del recinto (10 x 20 m, celdas de 10 cm) construido con los
// 16 infrarrojos y la pose. No depende de la luz: solo usa IR, GPS y brújula.
//
// Cada celda guarda una evidencia entera:
//   impacto IR en la celda  -> +kHitIncrement
//   rayo IR que la atraviesa sin detectar nada -> -kFreeDecrement
//   celda bajo el robot     -> mínimo (seguro que está libre)
// Ocupada si la evidencia >= kOccupiedThreshold. Desconocida = 0.
// Las paredes exteriores del recinto (dato del enunciado) se marcan desde el
// principio como ocupadas y no se pueden borrar.
//
// Además cuenta cuántas veces ha entrado el robot en cada celda: el
// planificador penaliza volver a zonas ya recorridas (memoria de por dónde pasó).
//
// Implementación solo en cabecera para no tener que tocar el Makefile.
class OccupancyGrid {
public:
  static constexpr double kCellM = robot_config::mapping::kCellM;
  static constexpr double kMinX = -5.0;
  static constexpr double kMinZ = -10.0;
  static constexpr int kWidth = 100;   // celdas en x
  static constexpr int kHeight = 200;  // celdas en z

  OccupancyGrid()
      : _value(kWidth * kHeight, 0), _visits(kWidth * kHeight, 0),
        _fixed(kWidth * kHeight, 0) {
    namespace m = robot_config::mapping;
    for (int cz = 0; cz < kHeight; ++cz)
      for (int cx = 0; cx < kWidth; ++cx) {
        double x, z;
        cellCenter(cx, cz, x, z);
        if (std::fabs(x) > m::kArenaHalfXM || std::fabs(z) > m::kArenaHalfZM) {
          _value[index(cx, cz)] = m::kMaxEvidence;
          _fixed[index(cx, cz)] = 1;
        }
      }
  }

  static int index(int cx, int cz) { return cz * kWidth + cx; }
  static bool inside(int cx, int cz) { return cx >= 0 && cx < kWidth && cz >= 0 && cz < kHeight; }

  static bool toCell(double x, double z, int &cx, int &cz) {
    cx = static_cast<int>(std::floor((x - kMinX) / kCellM));
    cz = static_cast<int>(std::floor((z - kMinZ) / kCellM));
    return inside(cx, cz);
  }
  static void cellCenter(int cx, int cz, double &x, double &z) {
    x = kMinX + (cx + 0.5) * kCellM;
    z = kMinZ + (cz + 0.5) * kCellM;
  }

  bool occupied(int cx, int cz) const {
    return !inside(cx, cz) ||
           _value[index(cx, cz)] >= robot_config::mapping::kOccupiedThreshold;
  }
  bool known(int cx, int cz) const { return inside(cx, cz) && _value[index(cx, cz)] != 0; }
  int visits(int cx, int cz) const { return inside(cx, cz) ? _visits[index(cx, cz)] : 0; }
  // Cambia cada vez que una celda pasa de libre a ocupada o al revés.
  int version() const { return _version; }
  int occupiedCount() const { return _occupiedCount; }

  // Integra las lecturas IR de un paso con la pose actual.
  void update(const SensorSnapshot &s, const Pose &pose) {
    namespace m = robot_config::mapping;
    const double fx = -std::sin(pose.yawRad), fz = -std::cos(pose.yawRad);  // delante
    const double lx = -std::cos(pose.yawRad), lz = std::sin(pose.yawRad);   // izquierda

    // 1. El robot está aquí: celdas bajo su huella libres; contar visitas.
    int rcx, rcz;
    if (toCell(pose.xM, pose.zM, rcx, rcz)) {
      const int cell = index(rcx, rcz);
      if (cell != _lastRobotCell) {
        if (_visits[cell] < 255) ++_visits[cell];
        _lastRobotCell = cell;
      }
    }
    const int r = static_cast<int>(std::ceil(m::kFootprintRadiusM / kCellM));
    for (int dz = -r; dz <= r; ++dz)
      for (int dx = -r; dx <= r; ++dx) {
        if ((dx * dx + dz * dz) * kCellM * kCellM > m::kFootprintRadiusM * m::kFootprintRadiusM)
          continue;
        setValue(rcx + dx, rcz + dz, m::kMinEvidence);
      }

    if (!s.distanceSensorsValid) return;
    // 2. Rayos de los infrarrojos.
    for (int i = 0; i < robot_config::kDistanceSensorCount; ++i) {
      const double a = kSensorAngleRad(i);
      const double dirF = std::cos(a), dirL = std::sin(a);
      const double rayX = fx * dirF + lx * dirL, rayZ = fz * dirF + lz * dirL;
      const double sx = pose.xM + fx * kSensorForwardM(i) + lx * kSensorLeftM(i);
      const double sz = pose.zM + fz * kSensorForwardM(i) + lz * kSensorLeftM(i);
      const bool hit = s.distanceDetected[i] && s.distanceM[i] < m::kIrMaxRangeM;
      const double freeUntil = hit ? s.distanceM[i] - 0.04 : m::kIrMaxRangeM;
      for (double t = 0.02; t < freeUntil; t += 0.05) addEvidence(sx + rayX * t, sz + rayZ * t, -m::kFreeDecrement);
      if (hit) {
        // El punto medido está en la superficie: empujar 3 cm hacia dentro.
        const double d = s.distanceM[i] + 0.03;
        addEvidence(sx + rayX * d, sz + rayZ * d, m::kHitIncrement);
      }
    }
  }

  // Volcado para el visor en directo (tools/live_map.py). Se escribe en un
  // archivo temporal y se renombra, así el visor nunca lee un archivo a medias.
  // Formato: líneas "clave=valor", luego "grid" y las filas de arriba (+z) a abajo.
  void writeLive(const std::string &path, double timeS, const std::string &state,
                 const Pose &pose, double goalZ,
                 const std::vector<std::pair<double, double>> &plan) const {
    const std::string tmp = path + ".tmp";
    {
      std::ofstream out(tmp);
      out << "t=" << timeS << "\nstate=" << state << "\npose=" << pose.xM << ' ' << pose.zM
          << ' ' << pose.yawRad << "\ngoalz=" << goalZ << "\nplan=";
      for (size_t i = 0; i < plan.size(); i += 4)  // un punto de cada 4 (40 cm)
        out << plan[i].first << ',' << plan[i].second << ' ';
      out << "\ngrid\n";
      for (int cz = kHeight - 1; cz >= 0; --cz) {
        for (int cx = 0; cx < kWidth; ++cx) {
          const int i = index(cx, cz);
          out << (occupied(cx, cz) ? '#' : _visits[i] > 0 ? 'o' : _value[i] < 0 ? '.' : ' ');
        }
        out << '\n';
      }
    }
    std::rename(tmp.c_str(), path.c_str());
  }

  // Dibujo ASCII para el informe: '#' ocupado, '.' libre visto, 'o' recorrido,
  // '*' último plan, ' ' desconocido. Arriba = +z (víctimas).
  void writeAscii(const std::string &path,
                  const std::vector<std::pair<double, double>> &plan) const {
    std::vector<char> canvas(kWidth * kHeight, ' ');
    for (int cz = 0; cz < kHeight; ++cz)
      for (int cx = 0; cx < kWidth; ++cx) {
        const int i = index(cx, cz);
        if (occupied(cx, cz)) canvas[i] = '#';
        else if (_visits[i] > 0) canvas[i] = 'o';
        else if (_value[i] < 0) canvas[i] = '.';
      }
    for (const auto &p : plan) {
      int cx, cz;
      if (toCell(p.first, p.second, cx, cz) && canvas[index(cx, cz)] != '#') canvas[index(cx, cz)] = '*';
    }
    std::ofstream out(path);
    out << "# Mapa de ocupacion (celda " << kCellM << " m). Arriba +z, izquierda -x.\n"
        << "# '#' ocupado  '.' libre  'o' recorrido  '*' ultimo plan  ' ' desconocido\n";
    for (int cz = kHeight - 1; cz >= 0; --cz) {
      for (int cx = 0; cx < kWidth; ++cx) out << canvas[index(cx, cz)];
      out << '\n';
    }
  }

private:
  // Geometría de los infrarrojos del Pioneer2 (proto): posición en el marco del
  // robot (delante, izquierda) y ángulo del rayo (positivo = izquierda).
  static double kSensorAngleRad(int i) {
    static const double deg[16] = {10, 30, 50, 90, 90, 130, 150, 170,
                                   -170, -150, -130, -90, -90, -50, -30, -10};
    return deg[i] * 3.14159265358979 / 180.0;
  }
  static double kSensorForwardM(int i) {
    static const double f[16] = {0.164, 0.147, 0.116, 0.071, -0.160, -0.205, -0.236, -0.253,
                                 -0.253, -0.236, -0.205, -0.160, 0.071, 0.116, 0.147, 0.164};
    return f[i];
  }
  static double kSensorLeftM(int i) {
    static const double l[16] = {0.027, 0.077, 0.118, 0.136, 0.136, 0.118, 0.077, 0.027,
                                 -0.027, -0.077, -0.118, -0.136, -0.136, -0.118, -0.077, -0.027};
    return l[i];
  }

  void addEvidence(double x, double z, int delta) {
    int cx, cz;
    if (!toCell(x, z, cx, cz)) return;
    const int i = index(cx, cz);
    // Una celda bajo el robot hace poco no se marca ocupada por un rayo rasante.
    setValue(cx, cz, _value[i] + delta);
  }

  void setValue(int cx, int cz, int value) {
    namespace m = robot_config::mapping;
    if (!inside(cx, cz)) return;
    const int i = index(cx, cz);
    if (_fixed[i]) return;
    const bool before = _value[i] >= m::kOccupiedThreshold;
    _value[i] = static_cast<int16_t>(std::max(m::kMinEvidence, std::min(m::kMaxEvidence, value)));
    const bool after = _value[i] >= m::kOccupiedThreshold;
    if (before != after) {
      ++_version;
      _occupiedCount += after ? 1 : -1;
    }
  }

  std::vector<int16_t> _value;
  std::vector<uint8_t> _visits;
  std::vector<uint8_t> _fixed;
  int _lastRobotCell = -1;
  int _version = 0;
  int _occupiedCount = 0;
};

#endif  // RESCUE_OCCUPANCY_GRID_H_
