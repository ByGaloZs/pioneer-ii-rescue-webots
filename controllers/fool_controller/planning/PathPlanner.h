#ifndef RESCUE_PATH_PLANNER_H_
#define RESCUE_PATH_PLANNER_H_

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "../config/RobotConfig.h"
#include "../localization/Localizer.h"
#include "../mapping/OccupancyGrid.h"

// A* sobre el mapa de ocupación hasta una línea meta (z >= zTarget hacia +z,
// o z <= zTarget hacia -z).
//  - Optimista: lo desconocido se considera libre. Al descubrir una pared el
//    mapa cambia y se replanifica (exploración tipo "D*-lite pobre").
//  - Celdas a menos de kLethalRadiusM de una ocupada: prohibidas (el robot no cabe).
//  - Coste extra cerca de obstáculos (ir por el centro de los pasos) y en
//    celdas ya recorridas (no repetir callejones).
// Devuelve la ruta en coordenadas del mundo y, para el navegador, la dirección
// hacia un punto de la ruta a ~kLookaheadM por delante.
//
// Implementación solo en cabecera para no tener que tocar el Makefile.
class PathPlanner {
public:
  using Point = std::pair<double, double>;  // (x, z)

  bool plan(const OccupancyGrid &grid, const Pose &pose, double zTarget, double direction) {
    namespace m = robot_config::mapping;
    constexpr int W = OccupancyGrid::kWidth, H = OccupancyGrid::kHeight;
    buildCostmap(grid);
    ++_plans;
    _path.clear();
    _valid = false;

    int sx, sz;
    if (!OccupancyGrid::toCell(pose.xM, pose.zM, sx, sz)) return false;
    const int start = OccupancyGrid::index(sx, sz);
    const double cell = OccupancyGrid::kCellM;
    const int startFree = static_cast<int>(std::ceil(m::kStartEscapeRadiusM / cell));

    auto isGoal = [&](int cz) {
      const double z = OccupancyGrid::kMinZ + (cz + 0.5) * cell;
      return (z - zTarget) * direction >= 0.0;
    };
    auto heuristic = [&](int cz) {
      const double z = OccupancyGrid::kMinZ + (cz + 0.5) * cell;
      return std::max(0.0, (zTarget - z) * direction);
    };

    const float inf = std::numeric_limits<float>::infinity();
    std::vector<float> g(W * H, inf);
    std::vector<int> parent(W * H, -1);
    std::vector<uint8_t> closed(W * H, 0);
    using Entry = std::pair<float, int>;
    std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
    g[start] = 0.0f;
    open.push({static_cast<float>(heuristic(sz)), start});

    static const int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    static const int kDz[8] = {0, 0, 1, -1, 1, -1, 1, -1};
    int goal = -1;
    while (!open.empty()) {
      const int current = open.top().second;
      open.pop();
      if (closed[current]) continue;
      closed[current] = 1;
      const int cx = current % W, cz = current / W;
      if (isGoal(cz) && !_lethal[current]) {
        goal = current;
        break;
      }
      for (int k = 0; k < 8; ++k) {
        const int nx = cx + kDx[k], nz = cz + kDz[k];
        if (!OccupancyGrid::inside(nx, nz)) continue;
        const int next = OccupancyGrid::index(nx, nz);
        if (closed[next] || grid.occupied(nx, nz)) continue;
        // Prohibidas, salvo para salir si el robot ya está dentro (pegado a algo).
        const bool nearStart = std::abs(nx - sx) <= startFree && std::abs(nz - sz) <= startFree;
        if (_lethal[next] && !nearStart) continue;
        // Diagonal: no cortar esquinas entre dos celdas ocupadas.
        if (k >= 4 && (grid.occupied(cx + kDx[k], cz) || grid.occupied(cx, cz + kDz[k]))) continue;
        const float step = static_cast<float>((k < 4 ? 1.0 : 1.41421356) * cell + _extra[next] +
                                              (_lethal[next] ? 1.0 : 0.0));
        if (g[current] + step < g[next]) {
          g[next] = g[current] + step;
          parent[next] = current;
          open.push({g[next] + static_cast<float>(heuristic(nz)), next});
        }
      }
    }
    if (goal < 0) return false;

    // Reconstruir y medir la longitud geométrica restante en cada punto.
    std::vector<int> cells;
    for (int c = goal; c >= 0; c = parent[c]) cells.push_back(c);
    std::reverse(cells.begin(), cells.end());
    _path.reserve(cells.size());
    for (int c : cells) {
      double x, z;
      OccupancyGrid::cellCenter(c % W, c / W, x, z);
      _path.push_back({x, z});
    }
    _remainingAt.assign(_path.size(), 0.0);
    for (int i = static_cast<int>(_path.size()) - 2; i >= 0; --i)
      _remainingAt[i] = _remainingAt[i + 1] + std::hypot(_path[i + 1].first - _path[i].first,
                                                         _path[i + 1].second - _path[i].second);
    _valid = true;
    return true;
  }

  bool valid() const { return _valid; }
  int plans() const { return _plans; }
  const std::vector<Point> &path() const { return _path; }

  // Longitud del camino que queda desde la pose (m).
  double remainingM(const Pose &pose) const {
    if (!_valid || _path.empty()) return 0.0;
    const int i = closestIndex(pose);
    return _remainingAt[i] +
           std::hypot(_path[i].first - pose.xM, _path[i].second - pose.zM);
  }

  // Orientación (convenio yaw del proyecto) hacia el punto de mira de la ruta:
  // el más lejano a <= kLookaheadM con línea de visión libre en el mapa.
  double lookaheadYaw(const Pose &pose) const {
    namespace m = robot_config::mapping;
    const int first = closestIndex(pose);
    int target = std::min(first + 1, static_cast<int>(_path.size()) - 1);
    for (int i = first + 1; i < static_cast<int>(_path.size()); ++i) {
      const double d = std::hypot(_path[i].first - pose.xM, _path[i].second - pose.zM);
      if (d > m::kLookaheadM) break;
      if (!lineOfSight(pose.xM, pose.zM, _path[i].first, _path[i].second)) break;
      target = i;
    }
    const double dx = _path[target].first - pose.xM, dz = _path[target].second - pose.zM;
    if (std::hypot(dx, dz) < 1e-6) return pose.yawRad;
    return std::atan2(-dx, -dz);  // avance = (-sin yaw, -cos yaw)
  }

private:
  int closestIndex(const Pose &pose) const {
    // La ruta empieza en el robot: buscar el más cercano en el primer tramo.
    int best = 0;
    double bestD = std::numeric_limits<double>::infinity();
    const int limit = std::min(static_cast<int>(_path.size()), 40);
    for (int i = 0; i < limit; ++i) {
      const double d = std::hypot(_path[i].first - pose.xM, _path[i].second - pose.zM);
      if (d < bestD) {
        bestD = d;
        best = i;
      }
    }
    return best;
  }

  bool lineOfSight(double x0, double z0, double x1, double z1) const {
    namespace m = robot_config::mapping;
    const double length = std::hypot(x1 - x0, z1 - z0);
    for (double t = m::kStartEscapeRadiusM; t < length; t += 0.05) {
      int cx, cz;
      if (!OccupancyGrid::toCell(x0 + (x1 - x0) * t / length, z0 + (z1 - z0) * t / length, cx, cz))
        return false;
      if (_lethal[OccupancyGrid::index(cx, cz)]) return false;
    }
    return true;
  }

  // Celdas prohibidas (radio del robot) y coste extra (cerca de obstáculos y
  // ya recorridas).
  void buildCostmap(const OccupancyGrid &grid) {
    namespace m = robot_config::mapping;
    constexpr int W = OccupancyGrid::kWidth, H = OccupancyGrid::kHeight;
    const double cell = OccupancyGrid::kCellM;
    _lethal.assign(W * H, 0);
    std::vector<float> nearest(W * H, static_cast<float>(m::kSoftRadiusM));
    const int r = static_cast<int>(std::ceil(m::kSoftRadiusM / cell));
    for (int cz = 0; cz < H; ++cz)
      for (int cx = 0; cx < W; ++cx) {
        if (!grid.occupied(cx, cz)) continue;
        for (int dz = -r; dz <= r; ++dz)
          for (int dx = -r; dx <= r; ++dx) {
            const int nx = cx + dx, nz = cz + dz;
            if (!OccupancyGrid::inside(nx, nz)) continue;
            const float d = static_cast<float>(std::sqrt(double(dx * dx + dz * dz)) * cell);
            const int i = OccupancyGrid::index(nx, nz);
            if (d < nearest[i]) nearest[i] = d;
          }
      }
    _extra.assign(W * H, 0.0f);
    for (int cz = 0; cz < H; ++cz)
      for (int cx = 0; cx < W; ++cx) {
        const int i = OccupancyGrid::index(cx, cz);
        if (nearest[i] <= m::kLethalRadiusM) _lethal[i] = 1;
        double extra = m::kSoftCostM * std::max(0.0, 1.0 - nearest[i] / m::kSoftRadiusM);
        extra += m::kVisitCostM * std::min(grid.visits(cx, cz), m::kMaxCountedVisits);
        _extra[i] = static_cast<float>(extra);
      }
  }

  std::vector<uint8_t> _lethal;
  std::vector<float> _extra;
  std::vector<Point> _path;
  std::vector<double> _remainingAt;
  bool _valid = false;
  int _plans = 0;
};

#endif  // RESCUE_PATH_PLANNER_H_
