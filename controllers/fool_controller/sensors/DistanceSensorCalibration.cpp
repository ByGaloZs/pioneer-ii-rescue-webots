#include "DistanceSensorCalibration.h"

#include <cmath>

namespace sensor_calibration {

DistanceSensorEstimate fromLookupTable(const double *lookupTable,
                                       int pointCount, double response,
                                       double reflectionFactor) {
  // Si los datos de entrada no son válidos se regresa la estimación vacía (sin detección).
  DistanceSensorEstimate estimate;
  if (lookupTable == 0 || pointCount < 2 ||
      !std::isfinite(reflectionFactor) || reflectionFactor <= 0.0 ||
      reflectionFactor > 1.0)
    return estimate;

  // La tabla es plana con 3 valores por fila: distancia, respuesta, ruido.
  const int last = (pointCount - 1) * 3;
  const double maximumRangeM = lookupTable[last];
  if (!std::isfinite(maximumRangeM) || maximumRangeM <= lookupTable[0])
    return estimate;
  estimate.distanceM = maximumRangeM;

  // Respuesta cero o negativa: no se ve nada, la distancia queda en el alcance máximo.
  if (!std::isfinite(response) || response <= 0.0)
    return estimate;

  estimate.detected = true;
  // Respuesta máxima (la fila más cercana de la tabla).
  const double maximumResponse = lookupTable[1];
  if (!std::isfinite(maximumResponse))
    return DistanceSensorEstimate();

  if (response >= maximumResponse) {
    // Una meseta en la respuesta no permite saber la distancia exacta de cerca.
    // Se reporta su borde lejano como límite de saturación, según la tabla del sensor.
    estimate.nearSaturated = true;
    estimate.nearSaturationLimitM = lookupTable[0] * reflectionFactor;
    for (int i = 1; i < pointCount; ++i) {
      const int current = i * 3;
      if (!std::isfinite(lookupTable[current]) ||
          !std::isfinite(lookupTable[current + 1]))
        return DistanceSensorEstimate();
      if (lookupTable[current + 1] != maximumResponse)
        break;
      estimate.nearSaturationLimitM = lookupTable[current] * reflectionFactor;
    }
    estimate.distanceM = estimate.nearSaturationLimitM;
    return estimate;
  }

  for (int i = 1; i < pointCount; ++i) {
    // Busca el tramo de la tabla que contiene la respuesta e interpola linealmente la distancia.
    const int nearIndex = (i - 1) * 3;
    const int farIndex = i * 3;
    const double nearDistance = lookupTable[nearIndex];
    const double farDistance = lookupTable[farIndex];
    const double nearResponse = lookupTable[nearIndex + 1];
    const double farResponse = lookupTable[farIndex + 1];
    if (!std::isfinite(nearDistance) || !std::isfinite(farDistance) ||
        !std::isfinite(nearResponse) || !std::isfinite(farResponse) ||
        farDistance <= nearDistance)
      return DistanceSensorEstimate();

    if (nearResponse > farResponse && response <= nearResponse &&
        response >= farResponse) {
      const double fraction =
          (nearResponse - response) / (nearResponse - farResponse);
      estimate.distanceM =
          (nearDistance + fraction * (farDistance - nearDistance)) *
          reflectionFactor;
      return estimate;
    }
  }

  // Una respuesta positiva fuera de la tabla se toma como saturación cercana.
  estimate.nearSaturated = true;
  estimate.nearSaturationLimitM = lookupTable[0] * reflectionFactor;
  estimate.distanceM = estimate.nearSaturationLimitM;
  return estimate;
}

}  // namespace sensor_calibration
