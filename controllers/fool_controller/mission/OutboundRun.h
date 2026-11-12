#ifndef RESCUE_OUTBOUND_RUN_H_
#define RESCUE_OUTBOUND_RUN_H_

#include <string>

#include "../control/SafetyGuard.h"
#include "../localization/Localizer.h"
#include "../mapping/OccupancyGrid.h"
#include "../navigation/GoalNavigator.h"
#include "../perception/FloorSegmenter.h"
#include "../sensors/SensorSnapshot.h"

// Prueba del paso 2: ir desde la salida hasta cruzar la línea amarilla del
// fondo (z >= 8.2) y medir el resultado. Al terminar imprime una línea
// [RESULT] y la añade a results/outbound_results.csv. Con mapa, guarda además
// el mapa de ocupación final en results/map_<tag>_<cam|nocam>.txt.
class OutboundRun {
public:
  OutboundRun(bool useCamera, bool useMap, bool liveMap, const std::string &tag);

  WheelCommand step(const SensorSnapshot &snapshot, const Localizer &localizer,
                    const FreeSpaceProfile *camera, double dtS);
  bool finished() const { return _finished; }
  void printStatus(const SensorSnapshot &snapshot, const Localizer &localizer) const;

private:
  void finish(const SensorSnapshot &snapshot, const Localizer &localizer, bool success);

  bool _useCamera;
  bool _useMap;
  bool _liveMap;
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

  double _startTimeS = 0.0;
  double _startX = 0.0, _startZ = 0.0, _startYaw = 0.0;
  double _maxProgressZ = -1e9;
  double _startTravelledM = 0.0;
  double _minIrM = 1e9;
  int _nearContacts = 0;
  bool _inNearContact = false;
};

#endif  // RESCUE_OUTBOUND_RUN_H_
