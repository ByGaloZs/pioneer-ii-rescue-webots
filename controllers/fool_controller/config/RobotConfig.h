#ifndef RESCUE_ROBOT_CONFIG_H_
#define RESCUE_ROBOT_CONFIG_H_

// Parámetros del robot en un único sitio.
// Valores de cinemática, brújula e IR ajustados con el informe que genera
// el modo de calibración (ver README.md, paso 1).

namespace robot_config {

// ---------------------------------------------------------------------------
// Nombres de dispositivos (Pioneer2 de los mundos final_scenarioXX.wbt)
// ---------------------------------------------------------------------------
constexpr const char *kLeftMotorName = "left wheel motor";
constexpr const char *kRightMotorName = "right wheel motor";
constexpr const char *kLeftEncoderName = "left wheel sensor";
constexpr const char *kRightEncoderName = "right wheel sensor";
constexpr const char *kCompassName = "compass";
constexpr const char *kGpsName = "gps";
constexpr const char *kFrontCameraName = "camera_f";      // fase final
constexpr const char *kSphericalCameraName = "camera_s";  // fase final

constexpr int kDistanceSensorCount = 16;

// ---------------------------------------------------------------------------
// Cinemática diferencial
// ---------------------------------------------------------------------------
constexpr double kMaximumWheelSpeedRadPerSec = 10.0;
// Radio de rueda. Calibrado 2026-11-12 (escenario 01): GPS 1.0375 m = encoders
// 1.0375 m en la recta -> el nominal es correcto.
constexpr double kWheelRadiusM = 0.0825;
// Separación EFECTIVA entre ruedas. La geométrica es 0.3206 m (ruedas en
// x = ±0.1603), pero la calibración midió 0.3606 m en giros de ±360° (el
// deslizamiento en los giros sobre sí mismo la hace parecer mayor).
constexpr double kWheelSeparationM = 0.3606;

// ---------------------------------------------------------------------------
// Brújula
// Convenio de orientación (yaw): el mismo ángulo que el campo "rotation 0 1 0 a"
// de Webots. yaw = atan2(c[2], c[0]).
//   yaw = pi    -> el robot mira hacia +Z (hacia las víctimas)
//   yaw = 0     -> el robot mira hacia -Z (hacia la salida)
//   yaw = +pi/2 -> mira hacia -X ;  yaw = -pi/2 -> mira hacia +X
// Girar a la izquierda (rueda derecha más rápida) hace crecer el yaw.
// Dirección de avance en el mundo: (dx, dz) = (-sin(yaw), -cos(yaw)).
// ---------------------------------------------------------------------------
// Calibración: desfase 0.00° entre brújula y dirección de avance del GPS.
constexpr double kCompassYawOffsetRad = 0.0;

// ---------------------------------------------------------------------------
// GPS
// Los mundos se configuran con GPS resolution -1 (sin cuantizar). Si el GPS no
// es fiable, poner false y la posición saldrá solo de la odometría.
// ---------------------------------------------------------------------------
constexpr bool kUseGpsForPosition = true;

// ---------------------------------------------------------------------------
// Sensores de distancia (16 infrarrojos, tabla nativa de 0.4 m)
// Webots R2020b atenúa la respuesta IR según el color del objeto:
// factor = 0.2 + 0.8 * rojo. Obstáculos y paredes cian -> rojo = 0.4 -> 0.52.
// Alcance efectivo resultante contra obstáculos: ~0.21 m.
// ---------------------------------------------------------------------------
// Factor de reflexión para obstáculos cian. Calibración: 0.527 medido
// (ds0 0.961 / ds15 1.015 de escala) -> se mantiene el teórico 0.52.
constexpr double kObstacleReflectionFactor = 0.52;
// Muestras de la mediana móvil por sensor (elimina picos de ruido).
constexpr int kDistanceMedianWindow = 3;

// ---------------------------------------------------------------------------
// Navegación (paso 2). Velocidades en rad/s de rueda (x 0.0825 = m/s).
// Distancias IR medidas desde cada sensor (alcance real ~0.20 m).
// ---------------------------------------------------------------------------
namespace nav {
constexpr double kOutboundGoalZ = 8.6;        // línea objetivo de la ida
constexpr double kOutboundDoneZ = 8.2;        // ida completada (línea amarilla en 8.0)
constexpr double kMissionTimeLimitS = 240.0;

constexpr double kAlignAngleRad = 1.05;       // >60° de error: girar en el sitio
constexpr double kHeadingGain = 4.0;          // diferencia de ruedas por rad de error
constexpr double kMaxHeadingCorrection = 2.0;

constexpr double kCruiseSpeedNoCamera = 4.0;  // ~0.33 m/s
constexpr double kNearObstacleSpeed = 2.5;    // con algún IR delantero detectando
constexpr double kCameraMinSpeed = 2.5;
constexpr double kCameraMaxSpeed = 8.0;       // ~0.66 m/s con el camino despejado
constexpr double kCameraSlowClearanceM = 0.8; // por debajo: velocidad mínima
constexpr double kCameraFastClearanceM = 2.5; // por encima: velocidad máxima
constexpr double kCameraSearchAngleRad = 0.35;// buscar huecos en ±20°
constexpr double kCameraNeededClearanceM = 2.0;
constexpr double kRobotHalfWidthM = 0.30;     // semiancho del pasillo (con margen)

constexpr double kHitFrontM = 0.18;           // ds0/ds15: obstáculo delante
constexpr double kHitDiagonalM = 0.15;        // ds1/ds14
constexpr double kSideRepulsionM = 0.14;      // ds2/ds3, ds12/ds13: apartarse
constexpr double kSideRepulsionGain = 15.0;

constexpr double kWallTargetM = 0.15;         // distancia lateral al rodear (sensor-pared)
constexpr double kWallFollowSpeed = 3.0;
constexpr double kWallDistanceGain = 15.0;
constexpr double kWallAlignGain = 12.0;
constexpr double kWallFrontBlockM = 0.18;
constexpr double kPivotSpeed = 2.2;
constexpr double kLeaveProgressM = 0.25;      // avance mínimo respecto al choque
constexpr double kRegressionLimitM = 1.5;     // retroceso que provoca media vuelta
constexpr double kFollowTimeoutS = 35.0;

constexpr double kSafetyStopM = 0.06;         // freno de emergencia (IR delanteros)
constexpr double kStuckWindowS = 1.5;
constexpr double kStuckMinMoveM = 0.03;
}  // namespace nav

// ---------------------------------------------------------------------------
// Mapa de ocupación y planificador A* (memoria de por dónde ha pasado).
// Solo usa IR + pose, así que no depende de la luz del escenario.
// ---------------------------------------------------------------------------
namespace mapping {
constexpr double kCellM = 0.1;               // celdas de 10 cm (100 x 200)
constexpr double kArenaHalfXM = 4.8;         // interior del recinto (enunciado: 10 x 20 m)
constexpr double kArenaHalfZM = 9.8;
constexpr double kIrMaxRangeM = 0.19;        // alcance IR de confianza contra cian
constexpr int kHitIncrement = 6;             // evidencia por impacto IR
constexpr int kFreeDecrement = 1;            // evidencia por rayo libre
constexpr int kOccupiedThreshold = 10;       // >= : ocupada (dos impactos)
constexpr int kMaxEvidence = 40;
constexpr int kMinEvidence = -20;
constexpr double kFootprintRadiusM = 0.15;   // celdas bajo el robot: libres

constexpr double kLethalRadiusM = 0.25;      // el robot no cabe más cerca de una celda ocupada
constexpr double kSoftRadiusM = 0.5;         // zona con coste extra junto a obstáculos
constexpr double kSoftCostM = 0.08;          // coste extra máximo por celda (m)
constexpr double kVisitCostM = 0.04;         // coste extra por cada visita previa (m)
constexpr int kMaxCountedVisits = 5;
constexpr double kStartEscapeRadiusM = 0.3;  // se permite salir de zona prohibida cerca del robot

constexpr double kLookaheadM = 0.8;          // punto de mira sobre la ruta
constexpr double kReplanPeriodS = 0.3;       // replanificar periódicamente
constexpr double kReplanMinPeriodS = 0.1;    // y al cambiar el mapa, como mucho cada 0.1 s
constexpr double kPlanBackwardRad = 2.36;    // ruta hacia atrás (>135°) rodeando ->
constexpr double kPlanBackwardTimeS = 1.0;   //   durante 1 s: media vuelta
constexpr double kPlanLeaveAfterS = 1.5;     // con plan, se puede soltar la pared tras 1.5 s
}  // namespace mapping

// ---------------------------------------------------------------------------
// Cámara frontal: robustez a la iluminación.
// ---------------------------------------------------------------------------
namespace camera {
constexpr int kMinFloorBrightness = 12;      // suelo más oscuro (0-255): cámara desactivada
constexpr double kObstacleRatio = 1.15;      // cian: g/r y b/r 15 % por encima de los del suelo
constexpr double kVictimRatio = 1.35;        // verde: g/r y g/b 35 % por encima de los del suelo
constexpr int kUnknownBrightness = 6;        // píxel más oscuro: no se sabe qué es (conservador)
constexpr double kTrustedRangeM = 3.5;       // más allá (niebla) no se confía en "libre"
}  // namespace camera

// ---------------------------------------------------------------------------
// Modo de arranque del controlador. Se puede forzar con controllerArgs del
// robot: "calibrate", "monitor", "navigate" y, con navigate, "nocam" para
// desactivar la ayuda de la cámara y "tag=XX" para etiquetar la prueba.
// ---------------------------------------------------------------------------
enum class StartupMode { kCalibrate, kMonitor, kNavigate };
constexpr StartupMode kStartupMode = StartupMode::kNavigate;
constexpr bool kUseCameraByDefault = true;
// Mapa + A*. Con "nomap" en controllerArgs se vuelve al Bug puro (para comparar).
constexpr bool kUseMapByDefault = true;
// Vuelca el mapa a results/live_map.txt (cada kLiveMapPeriodS de simulación)
// para verlo en directo con tools/live_map.py. "nolive" en controllerArgs lo apaga.
constexpr bool kLiveMapByDefault = true;
constexpr double kLiveMapPeriodS = 0.5;

}  // namespace robot_config

#endif  // RESCUE_ROBOT_CONFIG_H_
