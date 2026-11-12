// Controlador del Pioneer II para el trabajo final de Robótica.
//
// Modos (RobotConfig.h::kStartupMode o controllerArgs del robot en el .wbt):
//   navigate   paso 2: ir hasta la línea amarilla del fondo y medir el resultado.
//              Añadir "nocam" para navegar solo con infrarrojos, "nomap" para
//              desactivar el mapa + A* (Bug puro) y "tag=XX" para etiquetar a mano
//              (opcional: si no se pone, se detecta el mundo).
//   calibrate  paso 1: rutina que mide GPS, brújula, encoders e IR.
//   monitor    robot parado; imprime cada segundo pose y sectores de distancia.

#include <webots/Robot.hpp>

#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#include "calibration/SensorCalibration.h"
#include "common/Angles.h"
#include "config/RobotConfig.h"
#include "control/MotorController.h"
#include "localization/Localizer.h"
#include "mission/OutboundRun.h"
#include "perception/FloorSegmenter.h"
#include "sensors/SensorManager.h"

namespace {

struct Options {
  robot_config::StartupMode mode = robot_config::kStartupMode;
  bool useCamera = robot_config::kUseCameraByDefault;
  bool useMap = robot_config::kUseMapByDefault;
  std::string tag;
};

Options parseOptions(int argc, char **argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "calibrate") options.mode = robot_config::StartupMode::kCalibrate;
    else if (arg == "monitor") options.mode = robot_config::StartupMode::kMonitor;
    else if (arg == "navigate") options.mode = robot_config::StartupMode::kNavigate;
    else if (arg == "nocam") options.useCamera = false;
    else if (arg == "cam") options.useCamera = true;
    else if (arg == "nomap") options.useMap = false;
    else if (arg == "map") options.useMap = true;
    else if (arg.rfind("tag=", 0) == 0) options.tag = arg.substr(4);
  }
  return options;
}

void printSector(const char *name, const SectorReading &reading) {
  std::cout << ' ' << name << '=';
  if (reading.detected())
    std::cout << std::setprecision(3) << reading.distanceM;
  else
    std::cout << "--";
}

void printMonitor(const SensorSnapshot &snapshot, const Localizer &localizer) {
  const Pose &pose = localizer.pose();
  const Pose &odometry = localizer.odometryPose();
  std::cout << std::fixed << std::setprecision(1) << "[MONITOR] t=" << snapshot.timeS
            << std::setprecision(3) << " pose=(" << pose.xM << ", " << pose.zM << ") yaw="
            << std::setprecision(1) << angles::toDeg(pose.yawRad) << "deg"
            << std::setprecision(3) << " odom=(" << odometry.xM << ", " << odometry.zM << ")"
            << " gps=" << (snapshot.gpsValid ? "ok" : "NO")
            << " brujula=" << (snapshot.compassValid ? "ok" : "NO")
            << " encoders=" << (snapshot.encodersValid ? "ok" : "NO") << " | IR m:";
  printSector("F", snapshot.sectors.front);
  printSector("FL", snapshot.sectors.diagonalLeft);
  printSector("FR", snapshot.sectors.diagonalRight);
  printSector("L", snapshot.sectors.left);
  printSector("R", snapshot.sectors.right);
  printSector("BL", snapshot.sectors.rearLeft);
  printSector("BR", snapshot.sectors.rearRight);
  std::cout << std::endl;
}

}  // namespace

int main(int argc, char **argv) {
  webots::Robot robot;
  const int timeStep = static_cast<int>(robot.getBasicTimeStep());
  const double dtS = timeStep / 1000.0;

  MotorController motors(&robot);
  motors.initialize();
  motors.stop();

  SensorManager sensors(&robot);
  if (!sensors.initialize(timeStep)) {
    std::cerr << "[STARTUP] No se pudieron inicializar los sensores." << std::endl;
    return 1;
  }
  sensors.printDistanceSensorProfiles();

  const Options options = parseOptions(argc, argv);
  const bool navigate = options.mode == robot_config::StartupMode::kNavigate;
  std::cout << "[STARTUP] Modo: "
            << (navigate ? "navegación (ida)"
                         : options.mode == robot_config::StartupMode::kCalibrate ? "calibración"
                                                                                  : "monitor")
            << std::endl;

  // Cámara frontal solo si se navega con ella.
  std::unique_ptr<FloorSegmenter> segmenter;
  webots::Camera *frontCamera = nullptr;
  if (navigate && options.useCamera) {
    frontCamera = sensors.enableFrontCamera();
    if (frontCamera != nullptr)
      segmenter.reset(new FloorSegmenter(frontCamera->getWidth(), frontCamera->getHeight(),
                                         frontCamera->getFov()));
  }

  Localizer localizer;
  SensorCalibration calibration;
  OutboundRun outbound(options.useCamera && segmenter != nullptr, options.useMap, options.tag);
  double nextStatusS = 0.0;

  while (robot.step(timeStep) != -1) {
    const SensorSnapshot snapshot = sensors.read();
    localizer.update(snapshot);

    WheelCommand command = WheelCommand::stop();
    bool printNow = false;
    switch (options.mode) {
      case robot_config::StartupMode::kCalibrate:
        if (!calibration.finished()) command = calibration.step(snapshot, localizer);
        printNow = calibration.finished();
        break;
      case robot_config::StartupMode::kMonitor:
        printNow = true;
        break;
      case robot_config::StartupMode::kNavigate: {
        FreeSpaceProfile profile;
        if (segmenter != nullptr) profile = segmenter->process(frontCamera->getImage());
        command = outbound.step(snapshot, localizer, segmenter ? &profile : nullptr, dtS);
        if (!outbound.finished() && localizer.initialized() && snapshot.timeS >= nextStatusS) {
          nextStatusS = snapshot.timeS + 1.0;
          outbound.printStatus(snapshot, localizer);
        }
        break;
      }
    }
    motors.setWheelSpeeds(command.leftRadPerSec, command.rightRadPerSec);

    if (printNow && localizer.initialized() && snapshot.timeS >= nextStatusS) {
      nextStatusS = snapshot.timeS + 1.0;
      printMonitor(snapshot, localizer);
    }
  }

  motors.stop();
  sensors.shutdown();
  return 0;
}
