#ifndef RESCUE_DISTANCE_SENSOR_CALIBRATION_H_
#define RESCUE_DISTANCE_SENSOR_CALIBRATION_H_

// Resultado de convertir la respuesta de un sensor IR a distancia.
// detected: hay obstáculo; nearSaturated: está tan cerca que el sensor ya no distingue más;
// nearSaturationLimitM: distancia límite que se reporta en ese caso.
struct DistanceSensorEstimate {
  DistanceSensorEstimate()
      : distanceM(0.0), detected(false), nearSaturated(false),
        nearSaturationLimitM(0.0) {}

  double distanceM;
  bool detected;
  bool nearSaturated;
  double nearSaturationLimitM;
};

namespace sensor_calibration {

// Invierte la tabla [distancia, respuesta, ruido] activa de Webots y luego
// escala la distancia por el factor de reflexión IR supuesto. Ese factor depende
// del material; no se deduce de la respuesta del sensor.
// pointCount es el número de filas que regresa DistanceSensor::getLookupTableSize.
DistanceSensorEstimate fromLookupTable(const double *lookupTable, int pointCount,
                                      double response,
                                      double reflectionFactor);

}  // namespace sensor_calibration

#endif  // RESCUE_DISTANCE_SENSOR_CALIBRATION_H_
