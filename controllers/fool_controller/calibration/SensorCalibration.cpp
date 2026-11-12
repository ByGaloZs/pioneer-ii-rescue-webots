#include "SensorCalibration.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "../common/RunOutput.h"
#include "../common/Angles.h"
#include "../config/RobotConfig.h"

namespace {

// Parámetros de la rutina (no son parámetros del robot).
constexpr double kStaticDurationS = 2.0;
constexpr double kSettleDurationS = 0.6;
constexpr double kDriveSpeed = 3.0;             // rad/s de rueda (~0.25 m/s)
constexpr double kDriveDistanceM = 1.0;
constexpr double kDriveTimeoutS = 12.0;
constexpr double kDriveFrontStopM = 0.12;
constexpr double kHeadingGain = 4.0;
constexpr double kMaxHeadingCorrection = 1.5;
constexpr double kSpinSpeed = 2.0;
constexpr double kSpinTimeoutS = 20.0;
constexpr double kTurnToleranceRad = 1.5 * angles::kPi / 180.0;
constexpr double kTurnTimeoutS = 15.0;
constexpr double kApproachFastSpeed = 2.0;
constexpr double kApproachSlowSpeed = 0.6;      // ~0.05 m/s mientras se mide
constexpr double kApproachStopM = 0.06;
constexpr double kApproachMaxAfterDetectionM = 0.30;
constexpr double kApproachMaxTravelM = 4.0;
constexpr double kApproachTimeoutS = 45.0;
constexpr double kBackOffDistanceM = 0.30;
constexpr double kBackOffSpeed = 1.5;
constexpr double kFrontSensorAngleRad = 10.0 * angles::kPi / 180.0;  // ds0/ds15

const char *verdict(bool ok, bool warn) { return ok ? "OK" : (warn ? "REVISAR" : "FALLO"); }

double clamp(double value, double limit) { return std::max(-limit, std::min(limit, value)); }

}  // namespace

void SensorCalibration::enter(Phase phase, const SensorSnapshot &snapshot,
                              const Localizer &localizer) {
  _phase = phase;
  _phaseStartS = snapshot.timeS;
  _startX = snapshot.gpsXM;
  _startZ = snapshot.gpsZM;
  _startYaw = snapshot.compassYawRad;
  _startLeftRad = snapshot.leftEncoderRad;
  _startRightRad = snapshot.rightEncoderRad;
  _startAccumulatedYaw = localizer.accumulatedYawRad();
}

WheelCommand SensorCalibration::holdHeading(double speed, double targetYaw, double yaw) const {
  // Error positivo -> hay que aumentar el yaw -> girar a la izquierda.
  const double correction = clamp(kHeadingGain * angles::wrap(targetYaw - yaw),
                                  kMaxHeadingCorrection);
  return WheelCommand::of(speed - correction, speed + correction);
}

WheelCommand SensorCalibration::step(const SensorSnapshot &snapshot, const Localizer &localizer) {
  if (!localizer.initialized()) return WheelCommand::stop();
  if (!_started) {
    _started = true;
    _initialYaw = snapshot.compassYawRad;
    _yawRef = snapshot.compassYawRad;
    _restDetections.assign(robot_config::kDistanceSensorCount, 0);
    enter(Phase::kStatic, snapshot, localizer);
    std::cout << "[CALIB] Fase 1/5: reposo (" << kStaticDurationS << " s)" << std::endl;
  }
  const double elapsed = snapshot.timeS - _phaseStartS;
  const double yaw = snapshot.compassYawRad;

  switch (_phase) {
    case Phase::kStatic: {
      ++_staticSamples;
      _sumX += snapshot.gpsXM;
      _sumZ += snapshot.gpsZM;
      _sumX2 += snapshot.gpsXM * snapshot.gpsXM;
      _sumZ2 += snapshot.gpsZM * snapshot.gpsZM;
      const double dev = angles::wrap(yaw - _yawRef);
      _sumYawDev += dev;
      _sumYawDev2 += dev * dev;
      for (int i = 0; i < robot_config::kDistanceSensorCount; ++i)
        if (snapshot.distanceDetected[i]) ++_restDetections[i];
      if (elapsed >= kStaticDurationS) {
        enter(Phase::kDriveStraight, snapshot, localizer);
        _lastGpsX = snapshot.gpsXM;
        _lastGpsZ = snapshot.gpsZM;
        std::cout << "[CALIB] Fase 2/5: recta de " << kDriveDistanceM << " m" << std::endl;
      }
      return WheelCommand::stop();
    }

    case Phase::kDriveStraight: {
      ++_driveSteps;
      if (snapshot.gpsXM != _lastGpsX || snapshot.gpsZM != _lastGpsZ) ++_gpsChanges;
      _lastGpsX = snapshot.gpsXM;
      _lastGpsZ = snapshot.gpsZM;
      _driveSumSin += std::sin(yaw);
      _driveSumCos += std::cos(yaw);
      const double moved = std::hypot(snapshot.gpsXM - _startX, snapshot.gpsZM - _startZ);
      const bool blocked = snapshot.sectors.front.distanceM < kDriveFrontStopM;
      if (moved >= kDriveDistanceM || elapsed > kDriveTimeoutS || blocked) {
        if (blocked) std::cout << "[CALIB] Obstáculo delante: recta acortada." << std::endl;
        _phase = Phase::kSettleDrive;
        _phaseStartS = snapshot.timeS;
        return WheelCommand::stop();
      }
      return holdHeading(kDriveSpeed, _initialYaw, yaw);
    }

    case Phase::kSettleDrive:
      if (elapsed >= kSettleDurationS) {
        finishDrive(snapshot, localizer);
        enter(Phase::kSpinLeft, snapshot, localizer);
        std::cout << "[CALIB] Fase 3/5: giro de 360° a la izquierda y a la derecha" << std::endl;
      }
      return WheelCommand::stop();

    case Phase::kSpinLeft:
      if (localizer.accumulatedYawRad() - _startAccumulatedYaw >= 2.0 * angles::kPi ||
          elapsed > kSpinTimeoutS) {
        _phase = Phase::kSettleSpinLeft;
        _spinLeftDurationS = elapsed;
        _phaseStartS = snapshot.timeS;
        return WheelCommand::stop();
      }
      return WheelCommand::of(-kSpinSpeed, kSpinSpeed);

    case Phase::kSettleSpinLeft:
      if (elapsed >= kSettleDurationS) {
        finishSpin(true, snapshot, localizer);
        enter(Phase::kSpinRight, snapshot, localizer);
      }
      return WheelCommand::stop();

    case Phase::kSpinRight:
      if (localizer.accumulatedYawRad() - _startAccumulatedYaw <= -2.0 * angles::kPi ||
          elapsed > kSpinTimeoutS) {
        _phase = Phase::kSettleSpinRight;
        _spinRightDurationS = elapsed;
        _phaseStartS = snapshot.timeS;
        return WheelCommand::stop();
      }
      return WheelCommand::of(kSpinSpeed, -kSpinSpeed);

    case Phase::kSettleSpinRight:
      if (elapsed >= kSettleDurationS) {
        finishSpin(false, snapshot, localizer);
        enter(Phase::kTurnBack, snapshot, localizer);
        _approachYaw = angles::wrap(_initialYaw + angles::kPi);
        std::cout << "[CALIB] Fase 4/5: media vuelta y aproximación lenta a la pared" << std::endl;
      }
      return WheelCommand::stop();

    case Phase::kTurnBack: {
      const double error = angles::wrap(_approachYaw - yaw);
      if (std::fabs(error) < kTurnToleranceRad || elapsed > kTurnTimeoutS) {
        _phase = Phase::kSettleTurn;
        _phaseStartS = snapshot.timeS;
        return WheelCommand::stop();
      }
      double w = clamp(3.0 * error, 2.0);
      if (std::fabs(w) < 0.3) w = (w < 0 ? -0.3 : 0.3);
      return WheelCommand::of(-w, w);
    }

    case Phase::kSettleTurn:
      if (elapsed >= kSettleDurationS) enter(Phase::kApproach, snapshot, localizer);
      return WheelCommand::stop();

    case Phase::kApproach: {
      // Distancia recorrida proyectada sobre la dirección de avance inicial.
      const double fx = -std::sin(_startYaw), fz = -std::cos(_startYaw);
      const double travelled = (snapshot.gpsXM - _startX) * fx + (snapshot.gpsZM - _startZ) * fz;
      const bool d0 = snapshot.distanceDetected[0], d15 = snapshot.distanceDetected[15];
      if ((d0 || d15) && !_irDetectedOnce) {
        _irDetectedOnce = true;
        _firstDetectionTravelM = travelled;
      }
      if (_irDetectedOnce) {
        IrSample sample;
        sample.timeS = snapshot.timeS;
        sample.travelledM = travelled;
        sample.ds0M = snapshot.distanceM[0];
        sample.ds15M = snapshot.distanceM[15];
        sample.ds0Response = snapshot.distanceResponse[0];
        sample.ds15Response = snapshot.distanceResponse[15];
        sample.ds0Valid = d0 && !snapshot.distanceSaturated[0];
        sample.ds15Valid = d15 && !snapshot.distanceSaturated[15];
        _irSamples.push_back(sample);
      }
      const double closest = std::min(d0 ? snapshot.distanceM[0] : 1e9,
                                      d15 ? snapshot.distanceM[15] : 1e9);
      const bool saturated = snapshot.distanceSaturated[0] || snapshot.distanceSaturated[15];
      const bool tooFarAfterDetection =
          _irDetectedOnce && travelled - _firstDetectionTravelM > kApproachMaxAfterDetectionM;
      if (closest < kApproachStopM || saturated || tooFarAfterDetection) {
        _phase = Phase::kSettleApproach;
        _phaseStartS = snapshot.timeS;
        return WheelCommand::stop();
      }
      if (travelled > kApproachMaxTravelM || elapsed > kApproachTimeoutS) {
        _irSkippedReason = "no se detectó ninguna pared delante en el recorrido permitido";
        _phase = Phase::kSettleApproach;
        _phaseStartS = snapshot.timeS;
        return WheelCommand::stop();
      }
      const double speed = _irDetectedOnce ? kApproachSlowSpeed : kApproachFastSpeed;
      return holdHeading(speed, _startYaw, yaw);
    }

    case Phase::kSettleApproach:
      if (elapsed >= kSettleDurationS) {
        enter(Phase::kBackOff, snapshot, localizer);
        std::cout << "[CALIB] Fase 5/5: retroceso e informe" << std::endl;
      }
      return WheelCommand::stop();

    case Phase::kBackOff: {
      const double reversed = std::hypot(snapshot.gpsXM - _startX, snapshot.gpsZM - _startZ);
      if (reversed >= kBackOffDistanceM || elapsed > 5.0) {
        writeReport(snapshot, localizer);
        _phase = Phase::kDone;
        return WheelCommand::stop();
      }
      return WheelCommand::of(-kBackOffSpeed, -kBackOffSpeed);
    }

    case Phase::kDone:
      return WheelCommand::stop();
  }
  return WheelCommand::stop();
}

void SensorCalibration::finishDrive(const SensorSnapshot &snapshot, const Localizer &localizer) {
  const double dx = snapshot.gpsXM - _startX;
  const double dz = snapshot.gpsZM - _startZ;
  _gpsDistanceM = std::hypot(dx, dz);
  _encoderDistanceM = 0.5 * robot_config::kWheelRadiusM *
                      ((snapshot.leftEncoderRad - _startLeftRad) +
                       (snapshot.rightEncoderRad - _startRightRad));
  _motionYaw = std::atan2(-dx, -dz);
  _compassMeanYaw = std::atan2(_driveSumSin, _driveSumCos);
  _gpsChangeRatio = _driveSteps > 0 ? static_cast<double>(_gpsChanges) / _driveSteps : 0.0;
  _odometryErrorAfterDriveM = std::hypot(localizer.odometryPose().xM - snapshot.gpsXM,
                                         localizer.odometryPose().zM - snapshot.gpsZM);
  _driveDone = true;
}

void SensorCalibration::finishSpin(bool left, const SensorSnapshot &snapshot,
                                   const Localizer &localizer) {
  const double yawChange = localizer.accumulatedYawRad() - _startAccumulatedYaw;
  const double wheelDiff = robot_config::kWheelRadiusM *
                           ((snapshot.rightEncoderRad - _startRightRad) -
                            (snapshot.leftEncoderRad - _startLeftRad));
  if (left) {
    _spinLeftDone = true;
    _spinLeftYaw = yawChange;
    _spinLeftWheelDiffM = wheelDiff;
  } else {
    _spinRightDone = true;
    _spinRightYaw = yawChange;
    _spinRightWheelDiffM = wheelDiff;
  }
}

void SensorCalibration::analyseInfrared(std::ostringstream &out) {
  out << "[INFRARROJOS]\n";
  int restCount = 0;
  for (int i = 0; i < robot_config::kDistanceSensorCount; ++i)
    if (_restDetections[i] > 0) {
      if (restCount == 0) out << "  Detectan algo en reposo:";
      out << " ds" << i;
      ++restCount;
    }
  out << (restCount == 0 ? "  En reposo ningún sensor detecta nada (esperado lejos de paredes).\n"
                         : "\n");

  if (!_irSkippedReason.empty() || _irSamples.empty()) {
    out << "  Prueba de pared OMITIDA: "
        << (_irSkippedReason.empty() ? "sin muestras" : _irSkippedReason) << ".\n";
    return;
  }

  // CSV con las muestras brutas, por si hay que revisarlas.
  std::ofstream csv(run_output::path("calibration_ir_samples.csv"));
  csv << "time_s,travelled_m,ds0_response,ds0_m,ds0_valid,ds15_response,ds15_m,ds15_valid\n";
  for (const IrSample &s : _irSamples)
    csv << s.timeS << ',' << s.travelledM << ',' << s.ds0Response << ',' << s.ds0M << ','
        << s.ds0Valid << ',' << s.ds15Response << ',' << s.ds15M << ',' << s.ds15Valid << '\n';

  const IrSample &first = _irSamples.front();
  const double firstSeen = std::min(first.ds0Valid ? first.ds0M : 1e9,
                                    first.ds15Valid ? first.ds15M : 1e9);
  if (firstSeen < 1e8)
    out << "  Alcance efectivo contra pared cian: primera detección a " << std::setprecision(3)
        << firstSeen << " m del sensor.\n";

  // Ajuste lineal d_IR = a + b * recorrido. El rayo de ds0/ds15 va inclinado 10°,
  // así que si el factor es correcto b = -1/cos(10°).
  double factorSum = 0.0;
  int factorCount = 0;
  for (int sensor = 0; sensor < 2; ++sensor) {
    const bool isDs0 = sensor == 0;
    double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (const IrSample &s : _irSamples) {
      const bool valid = isDs0 ? s.ds0Valid : s.ds15Valid;
      if (!valid) continue;
      const double y = isDs0 ? s.ds0M : s.ds15M;
      n += 1; sx += s.travelledM; sy += y; sxx += s.travelledM * s.travelledM; sxy += s.travelledM * y;
    }
    const char *name = isDs0 ? "ds0 " : "ds15";
    const double denom = n * sxx - sx * sx;
    if (n < 5 || std::fabs(denom) < 1e-12) {
      out << "  " << name << ": muestras insuficientes (" << n << ").\n";
      continue;
    }
    const double b = (n * sxy - sx * sy) / denom;
    const double a = (sy - b * sx) / n;
    double residual2 = 0.0;
    for (const IrSample &s : _irSamples) {
      const bool valid = isDs0 ? s.ds0Valid : s.ds15Valid;
      if (!valid) continue;
      const double r = (isDs0 ? s.ds0M : s.ds15M) - (a + b * s.travelledM);
      residual2 += r * r;
    }
    const double noise = std::sqrt(residual2 / n);
    const double scale = -b * std::cos(kFrontSensorAngleRad);  // 1.0 = factor correcto
    const bool ok = std::fabs(scale - 1.0) < 0.10;
    const bool warn = std::fabs(scale - 1.0) < 0.25;
    out << "  " << name << ": " << static_cast<int>(n) << " muestras, pendiente " << std::setprecision(3)
        << b << " (ideal " << -1.0 / std::cos(kFrontSensorAngleRad) << "), escala medida "
        << scale << ", ruido residual " << noise << " m -> " << verdict(ok, warn) << "\n";
    if (scale > 0.2) {
      factorSum += robot_config::kObstacleReflectionFactor / scale;
      ++factorCount;
    }
  }
  if (factorCount > 0)
    out << "  Factor de reflexión medido: " << std::setprecision(3) << factorSum / factorCount
        << " (configurado " << robot_config::kObstacleReflectionFactor << ").\n";
}

void SensorCalibration::writeReport(const SensorSnapshot &snapshot, const Localizer &localizer) {
  std::ostringstream out;
  out << std::fixed;
  out << "\n================ INFORME DE CALIBRACIÓN ================\n";
  out << "Tiempo simulado: " << std::setprecision(1) << snapshot.timeS << " s\n";

  // --- GPS -----------------------------------------------------------------
  const double n = std::max(1, _staticSamples);
  const double sigmaX = std::sqrt(std::max(0.0, _sumX2 / n - (_sumX / n) * (_sumX / n)));
  const double sigmaZ = std::sqrt(std::max(0.0, _sumZ2 / n - (_sumZ / n) * (_sumZ / n)));
  out << "[GPS]\n" << std::setprecision(4)
      << "  Posición inicial: x=" << _sumX / n << " z=" << _sumZ / n << "\n"
      << "  Ruido en reposo: sigma_x=" << sigmaX << " m, sigma_z=" << sigmaZ << " m\n";
  const bool gpsSmooth = _gpsChangeRatio > 0.8;
  out << "  Durante la recta la posición cambió en el " << std::setprecision(0)
      << 100.0 * _gpsChangeRatio << "% de los pasos -> "
      << (gpsSmooth ? "OK (sin cuantizar)" : "FALLO: parece cuantizado, revisar GPS resolution -1")
      << "\n";

  // --- Brújula -------------------------------------------------------------
  const double yawMean = _sumYawDev / n;
  const double yawSigma = std::sqrt(std::max(0.0, _sumYawDev2 / n - yawMean * yawMean));
  const double conventionError = angles::wrap(_motionYaw - _compassMeanYaw);
  const double conventionDeg = std::fabs(angles::toDeg(conventionError));
  out << "[BRÚJULA]\n" << std::setprecision(2)
      << "  Yaw inicial: " << angles::toDeg(_initialYaw) << " deg (pi = mirando a +Z)\n"
      << "  Ruido en reposo: sigma=" << angles::toDeg(yawSigma) << " deg\n";
  if (_driveDone && _gpsDistanceM > 0.2) {
    out << "  Dirección de avance según GPS " << angles::toDeg(_motionYaw)
        << " deg, según brújula " << angles::toDeg(_compassMeanYaw) << " deg, diferencia "
        << angles::toDeg(conventionError) << " deg -> "
        << verdict(conventionDeg < 3.0, conventionDeg < 10.0) << "\n";
    if (conventionDeg > 150.0)
      out << "  El convenio está invertido: usar kCompassYawOffsetRad = pi.\n";
  } else {
    out << "  Convenio no comprobado: el robot no avanzó lo suficiente.\n";
  }
  const bool leftSignOk = _spinLeftDone && _spinLeftYaw > 5.0;
  const bool rightSignOk = _spinRightDone && _spinRightYaw < -5.0;
  out << "  Girar a la izquierda aumenta el yaw: " << (leftSignOk ? "sí" : "NO") << ", a la derecha lo reduce: "
      << (rightSignOk ? "sí" : "NO") << " -> " << verdict(leftSignOk && rightSignOk, false) << "\n";

  // --- Encoders --------------------------------------------------------------
  out << "[ENCODERS Y ODOMETRÍA]\n";
  double radius = robot_config::kWheelRadiusM;
  if (!gpsSmooth) {
    out << "  Radio no medido: el GPS está cuantizado y no sirve de referencia.\n";
  } else if (_driveDone && std::fabs(_encoderDistanceM) > 0.2) {
    const double ratio = _gpsDistanceM / _encoderDistanceM;
    radius = robot_config::kWheelRadiusM * ratio;
    const double pct = 100.0 * std::fabs(ratio - 1.0);
    out << std::setprecision(4) << "  Recta: GPS " << _gpsDistanceM << " m, encoders "
        << _encoderDistanceM << " m -> radio efectivo " << radius << " m (config "
        << robot_config::kWheelRadiusM << ") -> " << verdict(pct < 2.0, pct < 5.0) << "\n";
    if (_encoderDistanceM < 0)
      out << "  Los encoders cuentan negativo al avanzar: revisar el signo.\n";
  } else {
    out << "  Radio no medido: recta demasiado corta.\n";
  }
  double separationSum = 0.0;
  int separationCount = 0;
  const double radiusScale = radius / robot_config::kWheelRadiusM;
  if (leftSignOk) {
    const double l = _spinLeftWheelDiffM * radiusScale / _spinLeftYaw;
    separationSum += l; ++separationCount;
    out << std::setprecision(4) << "  Giro izquierda: " << angles::toDeg(_spinLeftYaw)
        << " deg en " << std::setprecision(1) << _spinLeftDurationS << " s -> separación efectiva "
        << std::setprecision(4) << l << " m\n";
  }
  if (rightSignOk) {
    const double l = _spinRightWheelDiffM * radiusScale / _spinRightYaw;
    separationSum += l; ++separationCount;
    out << std::setprecision(4) << "  Giro derecha: " << angles::toDeg(_spinRightYaw)
        << " deg en " << std::setprecision(1) << _spinRightDurationS << " s -> separación efectiva "
        << std::setprecision(4) << l << " m\n";
  }
  double separation = robot_config::kWheelSeparationM;
  if (separationCount > 0) {
    separation = separationSum / separationCount;
    const double pct = 100.0 * std::fabs(separation / robot_config::kWheelSeparationM - 1.0);
    out << "  Separación media " << separation << " m (config " << robot_config::kWheelSeparationM
        << ") -> " << verdict(pct < 3.0, pct < 8.0) << "\n";
  }
  out << std::setprecision(3) << "  Error de la odometría pura (sin GPS) tras la recta: "
      << _odometryErrorAfterDriveM << " m; al final de la rutina: "
      << std::hypot(localizer.odometryPose().xM - snapshot.gpsXM,
                    localizer.odometryPose().zM - snapshot.gpsZM)
      << " m\n";
  out << "  Diferencia yaw encoders vs brújula al final: " << std::setprecision(1)
      << angles::toDeg(angles::wrap(localizer.encoderYawRad() - snapshot.compassYawRad))
      << " deg (con la separación configurada)\n";

  // --- Infrarrojos -----------------------------------------------------------
  analyseInfrared(out);

  // --- Recomendaciones -------------------------------------------------------
  out << "[VALORES RECOMENDADOS PARA config/RobotConfig.h]\n" << std::setprecision(4)
      << "  kWheelRadiusM = " << radius << ";\n"
      << "  kWheelSeparationM = " << separation << ";\n";
  if (_driveDone && _gpsDistanceM > 0.2 && conventionDeg >= 3.0)
    out << "  kCompassYawOffsetRad = " << conventionError << ";\n";
  else
    out << "  kCompassYawOffsetRad = " << robot_config::kCompassYawOffsetRad << ";  (sin cambios)\n";
  out << "  kObstacleReflectionFactor: ver línea 'Factor de reflexión medido' arriba.\n";
  out << "========================================================\n";

  std::cout << out.str() << std::flush;
  std::ofstream file(run_output::path("calibration_report.txt"));
  file << out.str();
  std::cout << "[CALIB] Informe guardado en results/calibration_report.txt"
            << std::endl;
}
