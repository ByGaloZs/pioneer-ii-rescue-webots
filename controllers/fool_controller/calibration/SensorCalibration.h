#ifndef RESCUE_SENSOR_CALIBRATION_H_
#define RESCUE_SENSOR_CALIBRATION_H_

#include <sstream>
#include <string>
#include <vector>

#include "../control/WheelCommand.h"
#include "../localization/Localizer.h"
#include "../sensors/SensorSnapshot.h"

// Rutina automática de calibración. Pensada para lanzarse en final_scenario01
// desde la pose del archivo (0, -9.05) mirando a +Z, pero no usa la geometría
// del mundo: todo se mide con los propios sensores.
//
// Fases:
//  1. REPOSO       (2 s)  ruido de GPS y brújula, sensores que detectan algo.
//  2. RECTA        (~1 m) GPS vs encoders -> radio de rueda; dirección de avance
//                         del GPS vs brújula -> convenio de la brújula;
//                         cuantización del GPS.
//  3. GIRO 360 IZQ / DCHA  brújula vs encoders -> separación efectiva de ruedas
//                         y signo del giro.
//  4. MEDIA VUELTA y APROXIMACIÓN lenta a la pared que queda detrás
//                         -> distancia IR frente a distancia recorrida (GPS)
//                         -> factor de reflexión de los infrarrojos.
//  5. RETROCESO y INFORME (consola + calibration_report.txt).
class SensorCalibration {
public:
  // Un paso de la rutina: regresa las ruedas según la fase en curso.
  WheelCommand step(const SensorSnapshot &snapshot, const Localizer &localizer);
  // La rutina termina al llegar a kDone (ya con el informe escrito).
  bool finished() const { return _phase == Phase::kDone; }

private:
  // Fases de la rutina, cada una seguida de una pausa para estabilizar.
  enum class Phase {
    kStatic, kDriveStraight, kSettleDrive, kSpinLeft, kSettleSpinLeft, kSpinRight,
    kSettleSpinRight, kTurnBack, kSettleTurn, kApproach, kSettleApproach, kBackOff, kDone
  };

  // Una muestra de los IR frontales mientras el robot se acerca a la pared.
  struct IrSample {
    double timeS;
    double travelledM;
    double ds0M, ds15M;
    double ds0Response, ds15Response;
    bool ds0Valid, ds15Valid;
  };

  // Cambia de fase y guarda los datos de arranque.
  void enter(Phase phase, const SensorSnapshot &snapshot, const Localizer &localizer);
  // Avanza manteniendo el rumbo con corrección proporcional.
  WheelCommand holdHeading(double speed, double targetYaw, double yaw) const;
  // Calcula los resultados de la recta (distancia GPS vs encoders, rumbo).
  void finishDrive(const SensorSnapshot &snapshot, const Localizer &localizer);
  // Calcula los resultados de un giro de 360° (brújula vs encoders).
  void finishSpin(bool left, const SensorSnapshot &snapshot, const Localizer &localizer);
  // Imprime el informe y lo guarda en un archivo.
  void writeReport(const SensorSnapshot &snapshot, const Localizer &localizer);
  // Compara distancia IR contra distancia recorrida para sacar el factor de reflexión.
  void analyseInfrared(std::ostringstream &out);

  Phase _phase = Phase::kStatic;
  bool _started = false;
  double _phaseStartS = 0.0;

  // Fase de reposo.
  int _staticSamples = 0;
  double _sumX = 0, _sumZ = 0, _sumX2 = 0, _sumZ2 = 0;
  double _yawRef = 0, _sumYawDev = 0, _sumYawDev2 = 0;
  std::vector<int> _restDetections;

  // Datos al inicio de cada fase.
  double _startX = 0, _startZ = 0, _startYaw = 0;
  double _startLeftRad = 0, _startRightRad = 0;
  double _startAccumulatedYaw = 0;
  double _initialYaw = 0;

  // Recta.
  double _driveSumSin = 0, _driveSumCos = 0;
  int _driveSteps = 0, _gpsChanges = 0;
  double _lastGpsX = 0, _lastGpsZ = 0;

  // Resultados.
  bool _driveDone = false;
  double _gpsDistanceM = 0, _encoderDistanceM = 0, _motionYaw = 0, _compassMeanYaw = 0;
  double _odometryErrorAfterDriveM = 0;
  double _gpsChangeRatio = 0;
  bool _spinLeftDone = false, _spinRightDone = false;
  double _spinLeftYaw = 0, _spinRightYaw = 0;
  double _spinLeftWheelDiffM = 0, _spinRightWheelDiffM = 0;
  double _spinLeftDurationS = 0, _spinRightDurationS = 0;

  // Infrarrojos.
  double _approachYaw = 0;
  bool _irDetectedOnce = false;
  double _firstDetectionTravelM = 0;
  std::vector<IrSample> _irSamples;
  std::string _irSkippedReason;
};

#endif  // RESCUE_SENSOR_CALIBRATION_H_
