# Configuración (`config/RobotConfig.h`)

Todos los parámetros del robot están en un único archivo. Esta guía explica qué controla cada grupo y qué se nota al cambiarlo.
Las velocidades son rad/s de rueda (× 0,0825 = m/s; máximo 10 rad/s ≈ 0,83 m/s). Tras modificar el archivo hay que recompilar.

> **Regla de oro:** los valores de cinemática, brújula e IR salen de la calibración (ver [CALIBRACION.md](CALIBRACION.md)).
> No los cambies a mano sin volver a ejecutar `calibrate`. Los de `nav`, `mapping` y `camera` sí son ajustes de comportamiento.

## Dispositivos

| Constante | Valor | Qué es |
|---|---|---|
| `kLeftMotorName` / `kRightMotorName` | `left wheel motor` / `right wheel motor` | Motores de rueda |
| `kLeftEncoderName` / `kRightEncoderName` | `left wheel sensor` / `right wheel sensor` | Encoders |
| `kCompassName`, `kGpsName` | `compass`, `gps` | Orientación y posición |
| `kFrontCameraName` | `camera_f` | Cámara frontal (la usa la navegación) |
| `kSphericalCameraName` | `camera_s` | Cámara esférica (reservada para la fase de víctimas) |
| `kDistanceSensorCount` | 16 | Infrarrojos `ds0`…`ds15` |

## Cinemática diferencial

| Constante | Valor | Notas |
|---|---|---|
| `kMaximumWheelSpeedRadPerSec` | 10,0 | Saturación en `MotorController` |
| `kWheelRadiusM` | 0,0825 | Calibrado: GPS 1,0375 m = encoders 1,0375 m en la recta |
| `kWheelSeparationM` | 0,3606 | Separación **efectiva** (la geométrica es 0,3206): el deslizamiento al girar sobre sí mismo la hace parecer mayor |

## Brújula y GPS

| Constante | Valor | Notas |
|---|---|---|
| `kCompassYawOffsetRad` | 0,0 | Desfase brújula ↔ dirección de avance del GPS (medido 0,00°). Si saliera invertido, la calibración recomienda `π` |
| `kUseGpsForPosition` | `true` | `false` → la posición sale solo de la odometría (para una evaluación con GPS cuantizado a 3 m) |

## Infrarrojos

| Constante | Valor | Notas |
|---|---|---|
| `kObstacleReflectionFactor` | 0,52 | Webots atenúa la respuesta según el color: `0,2 + 0,8·rojo`. Con cian (rojo = 0,4) → 0,52. Medido 0,527 |
| `kDistanceMedianWindow` | 3 | Muestras de la mediana móvil por sensor |

## Navegación (`nav::`)

| Grupo | Constantes | Efecto |
|---|---|---|
| Meta y tiempo | `kOutboundGoalZ` 8,6; `kOutboundDoneZ` 8,2; `kMissionTimeLimitS` 240 | La meta del plan está más allá de la línea amarilla (z = 8) para que el robot la cruce; la prueba termina con éxito en z ≥ 8,2 o falla a los 240 s |
| Alineación | `kAlignAngleRad` 1,05 (60°); `kHeadingGain` 4,0; `kMaxHeadingCorrection` 2,0 | Con error > 60° gira en el sitio; si no, corrige el rumbo con ganancia proporcional limitada |
| Velocidades | `kCruiseSpeedNoCamera` 4,0; `kNearObstacleSpeed` 2,5; `kCameraMinSpeed` 2,5; `kCameraMaxSpeed` 8,0 | Crucero sin cámara; límite cuando un IR delantero detecta; rango con cámara |
| Cámara | `kCameraSlowClearanceM` 0,8; `kCameraFastClearanceM` 2,5; `kCameraSearchAngleRad` 0,35; `kCameraNeededClearanceM` 2,0; `kRobotHalfWidthM` 0,30 | Espacio libre para ir lento/rápido; abanico de búsqueda de ±20°; espacio libre que debe tener un rumbo; semiancho del pasillo (con margen) |
| Detección de obstáculo | `kHitFrontM` 0,18; `kHitDiagonalM` 0,15 | Distancias IR que disparan `FOLLOW_WALL` |
| Repulsión lateral | `kSideRepulsionM` 0,14; `kSideRepulsionGain` 15 | Apartarse de lo que roza al pasar |
| Seguir pared | `kWallTargetM` 0,15; `kWallFollowSpeed` 3,0; `kWallDistanceGain` 15; `kWallAlignGain` 12; `kWallFrontBlockM` 0,18; `kPivotSpeed` 2,2 | Distancia a la pared, velocidad y ganancias de seguimiento |
| Soltar la pared | `kLeaveProgressM` 0,25; `kRegressionLimitM` 1,5; `kFollowTimeoutS` 35 | Avance mínimo respecto al choque; retroceso que provoca media vuelta (sin mapa); tiempo máximo rodeando |
| Seguridad | `kSafetyStopM` 0,06 | Distancia IR delantera que anula el avance |
| Atasco | `kStuckWindowS` 1,5; `kStuckMinMoveM` 0,03 | Ventana de observación y movimiento mínimo esperado |

## Mapa y planificador (`mapping::`)

| Grupo | Constantes | Efecto |
|---|---|---|
| Rejilla | `kCellM` 0,1; `kArenaHalfXM` 4,8; `kArenaHalfZM` 9,8 | Tamaño de celda y límites del recinto (10 × 20 m); fuera de ellos las celdas son pared fija. La clase `OccupancyGrid` fija 100 × 200 celdas desde (−5, −10) |
| Evidencia | `kIrMaxRangeM` 0,19; `kHitIncrement` 6; `kFreeDecrement` 1; `kOccupiedThreshold` 10; `kMaxEvidence` 40; `kMinEvidence` −20 | Cuánto pesa un impacto o un rayo libre; con 6 de incremento, dos impactos marcan ocupada. Los límites evitan que una celda tarde demasiado en cambiar de opinión |
| Huella | `kFootprintRadiusM` 0,15 | Radio bajo el robot que se marca libre |
| Costes del A\* | `kLethalRadiusM` 0,25; `kSoftRadiusM` 0,5; `kSoftCostM` 0,08; `kVisitCostM` 0,04; `kMaxCountedVisits` 5; `kStartEscapeRadiusM` 0,3 | Radio donde el robot no cabe; zona de coste extra junto a paredes; penalización por visita previa; margen para salir de una zona prohibida si el robot ya está dentro |
| Seguimiento del plan | `kLookaheadM` 0,8; `kReplanPeriodS` 0,3; `kReplanMinPeriodS` 0,1 | Punto de mira y frecuencia de replanificación |
| Callejones | `kPlanBackwardRad` 2,36; `kPlanBackwardTimeS` 1,0; `kPlanLeaveAfterS` 1,5 | Si el plan pide ir atrás > 135° durante 1 s se deja la pared y se da media vuelta; con plan se puede soltar la pared tras 1,5 s |

Ajustes típicos: **más margen a las paredes** → subir `kLethalRadiusM` (cuidado: pasos estrechos pueden quedar prohibidos); **menos repetición de zonas** → subir `kVisitCostM`.

## Cámara (`camera::`)

| Constante | Valor | Efecto |
|---|---|---|
| `kMinFloorBrightness` | 12 | Si el suelo de referencia es más oscuro (0–255), la cámara se desactiva en ese fotograma |
| `kObstacleRatio` | 1,15 | Cian = g/r y b/r un 15 % por encima de los del suelo |
| `kVictimRatio` | 1,35 | Verde (víctima) = g/r y g/b un 35 % por encima del suelo |
| `kUnknownBrightness` | 6 | Píxel casi negro: se trata como desconocido (no cuenta como libre) |
| `kTrustedRangeM` | 3,5 | Más allá (niebla) no se da nada por libre |

Si un mundo nuevo es muy oscuro y la cámara se desactiva demasiado (`cam_valid_pct` bajo), bajar `kMinFloorBrightness` con cuidado: con poca luz el ruido produce
falsos obstáculos.

## Modo de arranque

| Constante | Valor | Equivalente en `controllerArgs` |
|---|---|---|
| `kStartupMode` | `kNavigate` | `calibrate`, `monitor`, `navigate` |
| `kUseCameraByDefault` | `true` | `nocam` / `cam` |
| `kUseMapByDefault` | `true` | `nomap` / `map` |

`controllerArgs` tiene prioridad sobre estos valores y no requiere recompilar.
