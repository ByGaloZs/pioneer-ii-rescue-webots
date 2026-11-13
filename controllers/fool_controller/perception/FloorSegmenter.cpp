#include "FloorSegmenter.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include "../config/RobotConfig.h"

namespace {

// Geometría de montaje de la cámara frontal y límites del análisis.
constexpr double kCameraHeightM = 0.255;   // altura de la cámara frontal en el robot
constexpr double kCameraForwardM = 0.155;  // distancia de la cámara frontal hacia delante del centro
constexpr double kMaxRangeM = 10.0;
constexpr int kObstacleRunPx = 2;          // píxeles seguidos para aceptar un obstáculo

}  // namespace

// Calcula la distancia focal en píxeles a partir del campo de visión.
FloorSegmenter::FloorSegmenter(int width, int height, double horizontalFovRad)
    : _width(width), _height(height),
      _focalPx(0.5 * width / std::tan(0.5 * horizontalFovRad)),
      _columnStep(std::max(1, width / 60)) {}

// Por geometría: entre más abajo del horizonte la fila, más cerca el suelo.
double FloorSegmenter::groundForwardM(int row) const {
  const double belowHorizonPx = row + 0.5 - 0.5 * _height;
  if (belowHorizonPx <= 0.0) return kMaxRangeM;
  return std::min(kMaxRangeM, kCameraHeightM * _focalPx / belowHorizonPx);
}

namespace {

// Clase de un píxel relativa al color del suelo de referencia.
enum class RelativeClass { kFloor, kObstacle, kVictim, kUnknown };

// Proporciones de color del suelo, que sirven de referencia para clasificar.
struct FloorReference {
  double greenOverRed = 1.0, blueOverRed = 1.0, greenOverBlue = 1.0;
  int brightness = 0;
};

// Clasifica un píxel comparando sus proporciones de color contra las del suelo.
RelativeClass classifyRelative(int r, int g, int b, const FloorReference &floor) {
  namespace c = robot_config::camera;
  const int brightest = std::max(r, std::max(g, b));
  if (brightest < c::kUnknownBrightness) return RelativeClass::kUnknown;
  const double rr = std::max(1, r), bb = std::max(1, b);
  const double gr = g / rr, br = b / rr, gb = g / bb;
  if (gr > c::kVictimRatio * floor.greenOverRed && gb > c::kVictimRatio * floor.greenOverBlue &&
      g >= std::max(robot_config::victims::kMinGreen,
                    static_cast<int>(robot_config::victims::kFloorGreenFactor * floor.brightness)))
    return RelativeClass::kVictim;
  if (gr > c::kObstacleRatio * floor.greenOverRed && br > c::kObstacleRatio * floor.blueOverRed)
    return RelativeClass::kObstacle;
  return RelativeClass::kFloor;  // gris, líneas negras/amarillas, sombras
}

int median(std::vector<int> &values) {
  if (values.empty()) return 0;
  std::nth_element(values.begin(), values.begin() + values.size() / 2, values.end());
  return values[values.size() / 2];
}

}  // namespace

// Barre cada columna de abajo hacia el horizonte y detecta dónde acaba el suelo libre.
FreeSpaceProfile FloorSegmenter::process(const unsigned char *image) const {
  namespace c = robot_config::camera;
  FreeSpaceProfile profile;
  if (image == nullptr) return profile;
  const int horizonRow = _height / 2;

  // 1. Suelo de referencia: franja inferior (8 %) del 50 % central.
  std::vector<int> reds, greens, blues;
  const int firstRow = _height - std::max(2, _height / 12);
  for (int row = firstRow; row < _height; ++row)
    for (int column = _width / 4; column < 3 * _width / 4; column += 2) {
      const unsigned char *pixel = image + 4 * (row * _width + column);  // BGRA
      reds.push_back(pixel[2]);
      greens.push_back(pixel[1]);
      blues.push_back(pixel[0]);
    }
  const int r0 = median(reds), g0 = median(greens), b0 = median(blues);
  FloorReference floor;
  floor.brightness = std::max(r0, std::max(g0, b0));
  profile.floorBrightness = floor.brightness;
  if (floor.brightness < c::kMinFloorBrightness) return profile;  // demasiado oscuro
  // Si la franja inferior ya es cian (pegado a un obstáculo), usar gris neutro.
  const double rr0 = std::max(1, r0), bb0 = std::max(1, b0);
  floor.greenOverRed = g0 / rr0;
  floor.blueOverRed = b0 / rr0;
  floor.greenOverBlue = g0 / bb0;
  if (floor.greenOverRed > 1.15 && floor.blueOverRed > 1.15)
    floor.greenOverRed = floor.blueOverRed = floor.greenOverBlue = 1.0;
  profile.valid = true;

  // Columnas consecutivas con víctima = una víctima. Se guarda el punto de la
  // base más cercano al robot (la base del cilindro es lo único que está en el suelo).
  std::vector<double> pendingForward, pendingLeft;
  // Cierra el grupo de columnas verdes y lo guarda como una víctima.
  auto closeBlob = [&]() {
    if (pendingForward.size() >= 2) {
      size_t best = 0;
      for (size_t i = 1; i < pendingForward.size(); ++i)
        if (std::hypot(pendingForward[i], pendingLeft[i]) <
            std::hypot(pendingForward[best], pendingLeft[best]))
          best = i;
      VictimBlob blob;
      blob.forwardM = pendingForward[best];
      blob.leftM = pendingLeft[best];
      blob.columns = static_cast<int>(pendingForward.size());
      profile.victims.push_back(blob);
    }
    pendingForward.clear();
    pendingLeft.clear();
  };

  // 2. Cada columna: de abajo hacia el horizonte hasta el primer no-suelo.
  for (int column = _columnStep / 2; column < _width; column += _columnStep) {
    int run = 0;
    int boundaryRow = -1;
    RelativeClass boundaryType = RelativeClass::kFloor;
    RelativeClass runType = RelativeClass::kFloor;
    for (int row = _height - 1; row > horizonRow; --row) {
      const unsigned char *pixel = image + 4 * (row * _width + column);  // BGRA
      const RelativeClass type = classifyRelative(pixel[2], pixel[1], pixel[0], floor);
      if (type == RelativeClass::kVictim) ++profile.victimPixels;
      if (type != RelativeClass::kFloor) {
        if (run == 0) runType = type;
        if (++run >= kObstacleRunPx) {
          boundaryType = runType;
          boundaryRow = row + kObstacleRunPx - 1;  // primera fila no-suelo
          break;
        }
      } else {
        run = 0;
      }
    }
    FreeSpacePoint point;
    point.closeObstacle = boundaryRow >= _height - 1;
    double cameraForward = boundaryRow < 0 ? kMaxRangeM : groundForwardM(boundaryRow);
    // Más allá del alcance de confianza (niebla, poca luz) no se asegura nada.
    // Si el verde llega al borde inferior de la imagen, la base queda fuera de
    // la imagen (< ~1 m): el punto no es la base, no sirve para localizarla.
    point.victim = boundaryRow >= 0 && boundaryRow < _height - 4 &&
                   boundaryType == RelativeClass::kVictim;
    point.rawForwardM = cameraForward + kCameraForwardM;
    cameraForward = std::min(cameraForward, c::kTrustedRangeM);
    const double offsetPx = column + 0.5 - 0.5 * _width;
    point.forwardM = cameraForward + kCameraForwardM;
    point.leftM = -cameraForward * offsetPx / _focalPx;
    if (point.victim) {  // punto real (sin recortar) para localizar la víctima
      const double rawLeft = -(point.rawForwardM - kCameraForwardM) * offsetPx / _focalPx;
      pendingForward.push_back(point.rawForwardM);
      pendingLeft.push_back(rawLeft);
    } else if (!pendingForward.empty()) {
      closeBlob();
    }
    profile.points.push_back(point);
  }
  if (!pendingForward.empty()) closeBlob();
  return profile;
}

// Para cada punto se calcula su avance y desvío respecto a la dirección dada;
// si cae dentro del pasillo, limita la distancia libre.
double FreeSpaceProfile::corridorClearance(double relativeHeadingRad, double halfWidthM,
                                           double maxRangeM) const {
  double clearance = maxRangeM;
  const double c = std::cos(relativeHeadingRad), s = std::sin(relativeHeadingRad);
  for (const FreeSpacePoint &p : points) {
    const double along = p.forwardM * c + p.leftM * s;
    const double across = -p.forwardM * s + p.leftM * c;
    if (along > 0.0 && std::fabs(across) < halfWidthM) clearance = std::min(clearance, along);
  }
  return clearance;
}
