#ifndef RESCUE_VICTIM_DETECTOR_H_
#define RESCUE_VICTIM_DETECTOR_H_

#include <algorithm>
#include <cmath>
#include <vector>

#include "../config/RobotConfig.h"
#include "../localization/Localizer.h"
#include "FloorSegmenter.h"
#include "SphericalView.h"

// Detección de víctimas (cilindros verdes de 0.6 m de diámetro y 0.5 m de alto).
//
//  - Cámara esférica (360°, mira al suelo): cada píxel verde se proyecta al suelo.
//    La base del cilindro es lo único que está en el suelo, así que el píxel
//    verde más cercano al robot en cada dirección da el borde cercano de la
//    víctima; el centro está 0.3 m más allá. Buena a < 3 m, en cualquier
//    dirección.
//  - Cámara frontal: FloorSegmenter ya devuelve el punto de la base más cercano
//    (preciso hasta ~5 m) para lo que hay delante.
//  - VictimTracker junta las observaciones de ambas en coordenadas del mundo
//    (media ponderada por la calidad de cada una) y lleva la cuenta de cuáles
//    están ya rescatadas, para no contar la misma dos veces.
//
// Implementación solo en cabecera para no tener que tocar el Makefile.

// Verde de víctima (0.16, 1, 0.055) bajo cualquier luz: el verde domina sobre rojo
// Y azul. El cian de las paredes tiene el azul alto; el suelo y las líneas no.
// minGreen: el verde de una víctima es MÁS brillante que el suelo gris (verde ~1.0
// frente a ~0.46): exigir g >= minGreen descarta el ruido sobre las líneas negras.
inline bool isVictimGreen(int r, int g, int b, int minGreen = 0) {
  if (std::max(r, std::max(g, b)) < 12 || g < minGreen) return false;
  return g > 1.5 * r + 6 && g > 1.5 * b + 6;
}

struct VictimObservation {
  double x, z;   // centro estimado del cilindro (mundo)
  double weight;
  double range;  // distancia robot -> base vista (m)
};

class VictimTracker {
public:
  struct Candidate {
    double x = 0.0, z = 0.0;
    double weight = 0.0;
    int frames = 0;       // imágenes en las que se ha visto
    int lastFrame = -1;
    int closeFrame = -1000;  // última imagen con una observación cercana (precisa)
    bool rescued = false;
    bool failed = false;  // intento de rescate fallido
  };

  void beginFrame() { ++_frame; }

  void add(const VictimObservation &o, double robotX, double robotZ) {
    namespace v = robot_config::victims;
    if (std::fabs(o.x) > v::kMaxAbsXM || o.z < v::kMinZ || o.z > v::kMaxZ) return;
    Candidate *best = nullptr;
    double bestD = v::kMergeRadiusM;
    for (Candidate &c : _candidates) {
      const double d = std::hypot(c.x - o.x, c.z - o.z);
      if (d < bestD) {
        bestD = d;
        best = &c;
      }
    }
    if (best == nullptr) {
      // Una candidata nueva pegada a una víctima ya vista de cerca (o rescatada)
      // son restos de ella (parte alta del cilindro que se proyecta más lejos).
      for (const Candidate &c : _candidates)
        if (c.rescued || c.closeFrame > -1000) {
          // Resto "detrás" de la víctima respecto al robot (la parte alta del
          // cilindro se proyecta más lejos, en la misma dirección de visión).
          const double dx = o.x - c.x, dz = o.z - c.z, d = std::hypot(dx, dz);
          const double vx = c.x - robotX, vz = c.z - robotZ, vl = std::max(0.05, std::hypot(vx, vz));
          if (d < v::kShadowRadiusM && (dx * vx + dz * vz) > 0.0 &&
              (dx * vx + dz * vz) / (std::max(0.05, d) * vl) > 0.8)
            return;
        }
      _candidates.push_back(Candidate());
      best = &_candidates.back();
    }
    if (best->rescued) return;  // ya rescatada: su posición ya es buena
    if (best->frames == 0) {
      best->x = o.x;
      best->z = o.z;
    } else if (o.range < v::kCloseRangeM) {
      // Una cercana es precisa: si cae lejos de la estimación es OTRA víctima
      // (o un resto de ella), no una corrección.
      if (std::hypot(best->x - o.x, best->z - o.z) > v::kCloseGateM) return;
      // Observación cercana (la base se ve a pocos píxeles de error): manda sobre
      // las lejanas, que tienen decenas de cm de error.
      best->x = (1.0 - v::kCloseBlend) * best->x + v::kCloseBlend * o.x;
      best->z = (1.0 - v::kCloseBlend) * best->z + v::kCloseBlend * o.z;
      best->closeFrame = _frame;
    } else if (_frame - best->closeFrame > v::kCloseHoldFrames) {
      const double total = best->weight + o.weight;
      best->x = (best->x * best->weight + o.x * o.weight) / total;
      best->z = (best->z * best->weight + o.z * o.weight) / total;
      best->weight = std::min(total, v::kMaxWeight);
    }
    if (o.range < v::kCloseRangeM) best->closeFrame = _frame;
    if (best->lastFrame != _frame) {
      ++best->frames;
      best->lastFrame = _frame;
    }
  }

  // Une candidatas que han acabado muy juntas (la misma víctima vista con
  // estimaciones lejanas distintas). Los índices cambian: llamar solo cuando no
  // hay un objetivo en curso.
  void mergeDuplicates() {
    for (size_t i = 0; i < _candidates.size(); ++i)
      for (size_t j = i + 1; j < _candidates.size();) {
        Candidate &a = _candidates[i], &b = _candidates[j];
        if (std::hypot(a.x - b.x, a.z - b.z) >= robot_config::victims::kDuplicateRadiusM) {
          ++j;
          continue;
        }
        const double wa = a.frames + 1, wb = b.frames + 1;
        a.x = (a.x * wa + b.x * wb) / (wa + wb);
        a.z = (a.z * wa + b.z * wb) / (wa + wb);
        a.frames += b.frames;
        a.rescued = a.rescued || b.rescued;
        a.failed = a.failed || b.failed;
        _candidates.erase(_candidates.begin() + j);
      }
  }

  bool confirmed(const Candidate &c) const {
    return c.frames >= robot_config::victims::kConfirmFrames;
  }

  // Candidato confirmado, no rescatado ni fallido, más cercano a (x, z); o nullptr.
  Candidate *nearestPending(double x, double z) {
    Candidate *best = nullptr;
    double bestD = 1e9;
    for (Candidate &c : _candidates) {
      if (c.rescued || c.failed || !confirmed(c)) continue;
      const double d = std::hypot(c.x - x, c.z - z);
      if (d < bestD) {
        bestD = d;
        best = &c;
      }
    }
    return best;
  }

  int rescuedCount() const {
    int n = 0;
    for (const Candidate &c : _candidates) n += c.rescued ? 1 : 0;
    return n;
  }
  int confirmedCount() const {
    int n = 0;
    for (const Candidate &c : _candidates) n += confirmed(c) ? 1 : 0;
    return n;
  }
  std::vector<Candidate> &candidates() { return _candidates; }
  const std::vector<Candidate> &candidates() const { return _candidates; }

private:
  std::vector<Candidate> _candidates;
  int _frame = 0;
};

namespace victim_detector {

constexpr double kCylinderRadiusM = 0.30;

inline void toWorld(const Pose &pose, double forward, double left, double &x, double &z) {
  const double fx = -std::sin(pose.yawRad), fz = -std::cos(pose.yawRad);
  const double lx = -std::cos(pose.yawRad), lz = std::sin(pose.yawRad);
  x = pose.xM + fx * forward + lx * left;
  z = pose.zM + fz * forward + lz * left;
}

// Punto de base más cercano (marco del robot) -> centro del cilindro (mundo).
inline VictimObservation centerFromBase(const Pose &pose, double forward, double left,
                                        double weight) {
  const double range = std::max(0.05, std::hypot(forward, left));
  const double f = forward + kCylinderRadiusM * forward / range;
  const double l = left + kCylinderRadiusM * left / range;
  VictimObservation o;
  toWorld(pose, f, l, o.x, o.z);
  o.weight = weight;
  o.range = range;
  return o;
}

// Cámara frontal: una observación por bloque verde.
inline void fromFront(const FreeSpaceProfile &profile, const Pose &pose,
                      std::vector<VictimObservation> &out) {
  namespace v = robot_config::victims;
  for (const VictimBlob &b : profile.victims) {
    const double range = std::hypot(b.forwardM, b.leftM);
    if (range > v::kFrontMaxRangeM || range < v::kFrontMinRangeM) continue;
    const double w = std::max(0.15, std::min(8.0, v::kFrontWeight / (range * range * range)));
    out.push_back(centerFromBase(pose, b.forwardM, b.leftM, w));
  }
}

// Cámara esférica: agrupa los píxeles verdes por dirección (bins de 3°), se queda
// con el más cercano de cada bin y une bins contiguos en un bloque.
// floorBrightness: brillo del suelo (0-255) medido por el mapa; <= 0 si no se sabe.
inline void fromSphere(const unsigned char *image, const Pose &pose, int floorBrightness,
                       std::vector<VictimObservation> &out) {
  namespace v = robot_config::victims;
  if (image == nullptr) return;
  constexpr int kBins = 120;
  const int minGreen = std::max(v::kMinGreen, static_cast<int>(1.15 * floorBrightness));
  double nearest[kBins];
  double nearestF[kBins], nearestL[kBins];
  int nearestU[kBins], nearestV[kBins];
  int count[kBins];
  for (int i = 0; i < kBins; ++i) {
    nearest[i] = 1e9;
    count[i] = 0;
  }
  for (int vv = 0; vv < SphericalView::kHeight; ++vv)
    for (int u = 0; u < SphericalView::kWidth; ++u) {
      const unsigned char *px = image + 4 * (vv * SphericalView::kWidth + u);  // BGRA
      if (!isVictimGreen(px[2], px[1], px[0], minGreen)) continue;
      if (SphericalView::isRobot(u, vv)) continue;
      double f, l;
      if (!SphericalView::pixelToGround(u, vv, v::kSphereMaxRangeM, f, l)) continue;
      const double range = std::hypot(f, l);
      double a = std::atan2(l, f);  // 0 = delante, positivo = izquierda
      if (a < 0) a += 2.0 * 3.14159265358979;
      const int bin = std::min(kBins - 1, static_cast<int>(a / (2.0 * 3.14159265358979) * kBins));
      ++count[bin];
      if (range < nearest[bin]) {
        nearest[bin] = range;
        nearestF[bin] = f;
        nearestL[bin] = l;
        nearestU[bin] = u;
        nearestV[bin] = vv;
      }
    }
  // Bins contiguos con base a distancias parecidas = un cilindro.
  bool used[kBins] = {false};
  for (int start = 0; start < kBins; ++start) {
    if (count[start] == 0 || used[start]) continue;
    // Retroceder hasta el primer bin del grupo (el grupo puede cruzar el bin 0).
    int first = start;
    while (count[(first + kBins - 1) % kBins] > 0 && !used[(first + kBins - 1) % kBins] &&
           (first + kBins - 1) % kBins != start)
      first = (first + kBins - 1) % kBins;
    std::vector<int> group;
    int i = first;
    while (count[i] > 0 && !used[i] && std::fabs(nearest[i] - nearest[first]) < 0.7) {
      used[i] = true;
      group.push_back(i);
      i = (i + 1) % kBins;
      if (i == first) break;
    }
    int pixels = 0;
    int best = group[0];
    for (int g : group) {
      pixels += count[g];
      if (nearest[g] < nearest[best]) best = g;
    }
    if (group.size() < 2 || pixels < v::kSphereMinPixels) continue;
    const double range = nearest[best];
    // Muy cerca la base queda en la zona ciega y el píxel más cercano es de más
    // arriba del cilindro (se proyecta más lejos): sesgado, mejor ignorarlo.
    if (range < v::kSphereMinRangeM) continue;
    // La base debe verse: hacia el robot (centro de la imagen) tiene que haber
    // suelo, no la silueta del propio robot. Si no, el píxel es de más arriba.
    bool baseVisible = true;
    for (int step = 3; step <= 9 && baseVisible; step += 3) {
      const double dxp = 76.0 - nearestU[best], dyp = 92.0 - nearestV[best];
      const double len = std::max(1.0, std::hypot(dxp, dyp));
      const int pu = static_cast<int>(nearestU[best] + dxp / len * step);
      const int pv = static_cast<int>(nearestV[best] + dyp / len * step);
      if (SphericalView::isRobot(pu, pv)) baseVisible = false;
    }
    if (!baseVisible) continue;
    const double w = std::max(0.1, std::min(4.0, v::kSphereWeight / (range * range * range)));
    out.push_back(centerFromBase(pose, nearestF[best], nearestL[best], w));
  }
}

}  // namespace victim_detector

#endif  // RESCUE_VICTIM_DETECTOR_H_
