#ifndef RESCUE_SPHERICAL_MAPPER_H_
#define RESCUE_SPHERICAL_MAPPER_H_

#include <algorithm>
#include <cmath>
#include <vector>

#include "../config/RobotConfig.h"
#include "../localization/Localizer.h"
#include "../perception/VictimDetector.h"
#include "../perception/SphericalView.h"
#include "OccupancyGrid.h"

// Mapa a partir de la cámara esférica (360°, mira al suelo).
//
// Para cada dirección alrededor del robot (cada 4°) recorre el suelo hacia fuera
// (0.5 m ... 1.6 m cada 5 cm). Cada punto del suelo se proyecta a su píxel con la
// geometría medida (SphericalView). Mientras el píxel sea suelo gris, el punto es
// espacio libre; el primer píxel cian (pared u obstáculo) marca una pared justo
// ahí y termina el rayo. Así se ve el mapa a 1-1.5 m en todas direcciones,
// también por los lados y por detrás, y no solo a 20 cm como los infrarrojos.
//
// Salvaguardas:
//  - Luz: se mide el brillo del suelo cerca del robot; por debajo de
//    kSphereMinBrightness (escenarios casi a oscuras) la imagen se ignora.
//  - Si la pared ya está dentro de la zona ciega (0.5 m) el primer punto es
//    cian y no se marca nada: de eso se encargan los infrarrojos.
//  - Se exigen kSphereMinFloorRun puntos de suelo antes de aceptar una pared.
//  - Un impacto suma poca evidencia (se necesitan varias imágenes seguidas).
//  - Nunca borra lo que han visto los infrarrojos.
//
// Implementación solo en cabecera para no tener que tocar el Makefile.
class SphericalMapper {
public:
  // image: buffer BGRA de webots::Camera::getImage() (160 x 160).
  // Devuelve false si la imagen no es utilizable (poca luz).
  // Lanza rayos en 360° sobre el suelo; el suelo libre se marca como libre
  // y donde empieza lo cian se marca una pared.
  bool update(const unsigned char *image, const Pose &pose, OccupancyGrid &grid) {
    namespace m = robot_config::mapping;
    if (image == nullptr) return false;
    _lastBrightness = floorBrightness(image);
    if (_lastBrightness >= 0 && _lastBrightness < m::kSphereMinBrightness) return false;

    // Vectores unitarios de "delante" e "izquierda" del robot en el mundo.
    const double fx = -std::sin(pose.yawRad), fz = -std::cos(pose.yawRad);  // delante
    const double lx = -std::cos(pose.yawRad), lz = std::sin(pose.yawRad);   // izquierda
    for (double deg = 0.0; deg < 360.0; deg += m::kSphereRayStepDeg) {
      const double a = deg * kPi / 180.0;
      const double ca = std::cos(a), sa = std::sin(a);
      int floorRun = 0;
      double lastFloorR = 0.0;
      for (double r = m::kSphereMinRangeM; r <= m::kSphereMaxRangeM + 1e-9;
           r += m::kSphereRangeStepM) {
        const double fwd = r * ca, left = r * sa;
        int u, v;
        if (!groundToPixel(fwd, left, u, v)) break;
        if (SphericalView::isRobot(u, v)) continue;  // lo tapa el propio robot
        const unsigned char *px = image + 4 * (v * SphericalView::kWidth + u);  // BGRA
        const bool obstacle = isObstacle(px[2], px[1], px[0]);
        if (!obstacle) {
          ++floorRun;
          lastFloorR = r;
          grid.addCameraFree(pose.xM + fx * fwd + lx * left, pose.zM + fz * fwd + lz * left);
          continue;
        }
        if (floorRun >= m::kSphereMinFloorRun) {
          // La base de la pared está entre el último suelo y este punto.
          const double hitR = 0.5 * (lastFloorR + r);
          const double hf = hitR * ca, hl = hitR * sa;
          grid.addCameraHit(pose.xM + fx * hf + lx * hl, pose.zM + fz * hf + lz * hl,
                            hitR <= 1.0 ? m::kSphereHitNear : m::kSphereHitFar);
        }
        break;
      }
    }
    return true;
  }

  // Brillo del suelo de la última imagen (para diagnóstico).
  int lastFloorBrightness() const { return _lastBrightness; }

private:
  static constexpr double kPi = 3.14159265358979;

  // Cian (paredes, obstáculos) o verde (víctimas): verde claramente por encima del
  // rojo. El suelo es gris (r ≈ g ≈ b), las líneas negras y amarillas no cumplen.
  static bool isObstacle(int r, int g, int b) {
    const int brightest = std::max(r, std::max(g, b));
    if (brightest < 10) return false;  // línea negra del suelo
    if (isVictimGreen(r, g, b)) return false;  // las víctimas no son paredes
    return g > 1.2 * r + 4 && (b > 1.2 * r + 4 || g > 1.4 * b);
  }

  // Punto del suelo (metros desde el centro del robot) -> píxel de la esférica.
  // Proyecta un punto del suelo al píxel de la cámara; false si cae fuera de la imagen.
  static bool groundToPixel(double forwardM, double leftM, int &u, int &v) {
    const double right = -leftM - SphericalView::kCameraRightM;
    const double fwd = forwardM - SphericalView::kCameraForwardM;
    const double down = SphericalView::kCameraHeightM;
    const double length = std::sqrt(right * right + fwd * fwd + down * down);
    const double dr = right / length, df = fwd / length, dd = down / length;
    const double phi = std::asin(std::max(-1.0, std::min(1.0, df)));
    const double lambda = std::atan2(dr, dd);
    const double k = SphericalView::kWidth / SphericalView::kFov;
    u = static_cast<int>(std::floor(0.5 * SphericalView::kWidth + lambda * k));
    v = static_cast<int>(std::floor(0.5 * SphericalView::kHeight - phi * k));
    return u >= 0 && u < SphericalView::kWidth && v >= 0 && v < SphericalView::kHeight;
  }

  // Brillo (0-255) mediano del suelo visible cerca del robot; -1 si no se puede medir.
  // Toma muestras del suelo alrededor del robot y regresa su brillo mediano.
  static int floorBrightness(const unsigned char *image) {
    std::vector<int> values;
    for (double deg = 0.0; deg < 360.0; deg += 20.0)
      for (double r : {0.55, 0.8, 1.05}) {
        const double a = deg * kPi / 180.0;
        int u, v;
        if (!groundToPixel(r * std::cos(a), r * std::sin(a), u, v)) continue;
        if (SphericalView::isRobot(u, v)) continue;
        const unsigned char *px = image + 4 * (v * SphericalView::kWidth + u);
        if (isObstacle(px[2], px[1], px[0])) continue;
        values.push_back(std::max(px[0], std::max(px[1], px[2])));
      }
    if (values.size() < 10) return -1;
    std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
    return values[values.size() / 2];
  }

  int _lastBrightness = -1;
};

#endif  // RESCUE_SPHERICAL_MAPPER_H_
