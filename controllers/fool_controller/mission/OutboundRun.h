#ifndef RESCUE_OUTBOUND_RUN_H_
#define RESCUE_OUTBOUND_RUN_H_

#include <string>

#include "../control/SafetyGuard.h"
#include "../localization/Localizer.h"
#include "../mapping/OccupancyGrid.h"
#include "../mapping/SphericalMapper.h"
#include "../navigation/GoalNavigator.h"
#include "../perception/FloorSegmenter.h"
#include "../sensors/SensorSnapshot.h"

// Prueba de la ida: ir desde la salida hasta cruzar la línea amarilla del
// fondo (z >= 8.2) y medir el resultado. Al terminar imprime una línea
// [RESULT] y la añade a results/outbound_results.csv. Con mapa, guarda además
// el mapa de ocupación final en results/map_<tag>_<cam|nocam>.txt.
class OutboundRun {
public:
  // Las banderas activan cámara, mapa, mapa en vivo y cámara esférica.
  OutboundRun(bool useCamera, bool useMap, bool liveMap, bool useSphere, const std::string &tag);

  // sphereImage: imagen BGRA de la cámara esférica (o nullptr).
  WheelCommand step(const SensorSnapshot &snapshot, const Localizer &localizer,
                    const FreeSpaceProfile *camera, const unsigned char *sphereImage,
                    double dtS);
  bool finished() const { return _finished; }
  void printStatus(const SensorSnapshot &snapshot, const Localizer &localizer) const;

private:
  // Cierra la prueba: imprime y guarda el resultado.
  void finish(const SensorSnapshot &snapshot, const Localizer &localizer, bool success);

  bool _useCamera;
  bool _useMap;
  bool _liveMap;
  bool _useSphere;
  // Contadores del mapa esférico y de la cámara (para el informe).
  SphericalMapper _sphereMapper;
  int _sphereFrames = 0;
  int _sphereValidFrames = 0;
  int _sphereStep = 0;
  bool _sphereLastOk = false;
  double _nextLiveS = 0.0;
  OccupancyGrid _grid;
  int _cameraFrames = 0;
  int _cameraValidFrames = 0;
  int _lastFloorBrightness = -1;
  std::string _tag;
  bool _started = false;
  bool _finished = false;
  GoalNavigator _navigator;
  SafetyGuard _safety;

  // Datos de arranque y métricas de la prueba.
  double _startTimeS = 0.0;
  double _startX = 0.0, _startZ = 0.0, _startYaw = 0.0;
  double _maxProgressZ = -1e9;
  double _startTravelledM = 0.0;
  double _minIrM = 1e9;
  int _nearContacts = 0;
  bool _inNearContact = false;
};

#endif  // RESCUE_OUTBOUND_RUN_H_
