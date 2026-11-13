// Controlador del Pioneer II para el trabajo final de Robótica.
//
// Modos (RobotConfig.h::kStartupMode o controllerArgs del robot en el .wbt):
//   mission    (por defecto) misión completa: ida, buscar y rescatar las 2 víctimas
//              (parar < 1 m, giro de 360°, mensaje) y volver a la salida. Acepta las
//              mismas opciones que navigate (nocam, nomap, nosphere, nolive, tag=XX).
//   navigate   paso 2: ir hasta la línea amarilla del fondo y medir el resultado.
//              Añadir "nocam" para navegar solo con infrarrojos, "nomap" para
//              desactivar el mapa + A* (Bug puro), "nosphere" para no usar la
//              cámara esférica en el mapa, "nolive" para no volcar el mapa
//              en directo (results/live_map.txt, ver tools/live_map.py) y "tag=XX" para etiquetar a mano
//              (opcional: si no se pone, se detecta el mundo).
//   calibrate  paso 1: rutina que mide GPS, brújula, encoders e IR.
//   capture    robot parado: guarda imágenes de las dos cámaras con su pose en
//              results/captures/ cada vez que lo arrastras a otro sitio. Con
//              "snap" también se guardan al navegar (cada 2 s, solo si se ha movido).
//   monitor    robot parado; imprime cada segundo pose y sectores de distancia.

#include <webots/Robot.hpp>

#include <cmath>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#include "calibration/SensorCalibration.h"
#include "capture/SnapshotRecorder.h"
#include "perception/SphericalView.h"
#include "common/Angles.h"
#include "config/RobotConfig.h"
#include "control/MotorController.h"
#include "localization/Localizer.h"
#include "mission/OutboundRun.h"
#include "mission/RescueMission.h"
#include "perception/FloorSegmenter.h"
#include "sensors/SensorManager.h"

namespace {

struct Options {
  robot_config::StartupMode mode = robot_config::kStartupMode;
  bool useCamera = robot_config::kUseCameraByDefault;
  bool useMap = robot_config::kUseMapByDefault;
  bool liveMap = robot_config::kLiveMapByDefault;
  bool useSphere = robot_config::kUseSphereByDefault;
  bool snap = false;  // guardar capturas de las cámaras también al navegar
  std::string tag;
};

Options parseOptions(int argc, char **argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "calibrate") options.mode = robot_config::StartupMode::kCalibrate;
    else if (arg == "monitor") options.mode = robot_config::StartupMode::kMonitor;
    else if (arg == "mission") options.mode = robot_config::StartupMode::kMission;
    else if (arg == "navigate") options.mode = robot_config::StartupMode::kNavigate;
    else if (arg == "capture") options.mode = robot_config::StartupMode::kCapture;
    else if (arg == "snap") options.snap = true;
    else if (arg == "nosphere") options.useSphere = false;
    else if (arg == "sphere") options.useSphere = true;
    else if (arg == "nocam") options.useCamera = false;
    else if (arg == "cam") options.useCamera = true;
    else if (arg == "nomap") options.useMap = false;
    else if (arg == "map") options.useMap = true;
    else if (arg == "nolive") options.liveMap = false;
    else if (arg == "live") options.liveMap = true;
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
  std::cout << "[STARTUP] controllerArgs recibidos (" << argc - 1 << "):";
  for (int i = 1; i < argc; ++i) std::cout << " [" << argv[i] << "]";
  std::cout << std::endl;
  const bool mission = options.mode == robot_config::StartupMode::kMission;
  const bool navigate = options.mode == robot_config::StartupMode::kNavigate || mission;
  const bool capture = options.mode == robot_config::StartupMode::kCapture;
  std::cout << "[STARTUP] Modo: "
            << (mission ? "misión completa (ida, víctimas, vuelta)"
                : navigate ? "navegación (ida)"
                : options.mode == robot_config::StartupMode::kCalibrate ? "calibración"
                : capture ? "captura de cámaras (robot parado: arrástralo)"
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

  // Capturas de las dos cámaras con la pose (modo "capture", o "snap" al navegar).
  std::unique_ptr<SnapshotRecorder> recorder;
  if (capture || (navigate && options.snap)) {
    webots::Camera *front = frontCamera != nullptr ? frontCamera : sensors.enableFrontCamera();
    webots::Camera *sphere = sensors.enableSphericalCamera();
    recorder.reset(new SnapshotRecorder(front, sphere, robot_config::kCapturePeriodS,
                                        /*onlyIfMoved=*/true));
  }

  // Cámara esférica para el mapa (solo al navegar con mapa).
  webots::Camera *sphereCamera = nullptr;
  if (navigate && options.useSphere && (options.useMap || mission)) {
    sphereCamera = sensors.enableSphericalCamera();
    if (sphereCamera != nullptr && !SphericalView::matches(sphereCamera->getWidth(),
                                                          sphereCamera->getHeight(),
                                                          sphereCamera->getFov())) {
      std::cerr << "[SENSORS] La cámara esférica no tiene la geometría medida (160x160, 180°); "
                   "no se usa para el mapa." << std::endl;
      sphereCamera = nullptr;
    }
  }

  Localizer localizer;
  SensorCalibration calibration;
  RescueMission rescue(options.useCamera && segmenter != nullptr, options.useMap, options.liveMap, sphereCamera != nullptr, options.tag);
  OutboundRun outbound(options.useCamera && segmenter != nullptr, options.useMap, options.liveMap, sphereCamera != nullptr, options.tag);
  double nextStatusS = 0.0;

  while (robot.step(timeStep) != -1) {
    const SensorSnapshot snapshot = sensors.read();
    localizer.update(snapshot);

    WheelCommand command = WheelCommand::stop();
    bool printNow = false;
    if (recorder && localizer.initialized()) recorder->update(snapshot.timeS, localizer.pose());
    switch (options.mode) {
      case robot_config::StartupMode::kCalibrate:
        if (!calibration.finished()) command = calibration.step(snapshot, localizer);
        printNow = calibration.finished();
        break;
      case robot_config::StartupMode::kMonitor:
        printNow = true;
        break;
      case robot_config::StartupMode::kCapture:
        break;  // robot parado; el recorder guarda una captura cada vez que se mueve
      case robot_config::StartupMode::kMission: {
        FreeSpaceProfile profile;
        if (segmenter != nullptr) profile = segmenter->process(frontCamera->getImage());
        command = rescue.step(snapshot, localizer, segmenter ? &profile : nullptr,
                              sphereCamera ? sphereCamera->getImage() : nullptr, dtS);
        if (!rescue.finished() && localizer.initialized() && snapshot.timeS >= nextStatusS) {
          nextStatusS = snapshot.timeS + 1.0;
          rescue.printStatus(snapshot, localizer);
        }
        break;
      }
      case robot_config::StartupMode::kNavigate: {
        FreeSpaceProfile profile;
        if (segmenter != nullptr) profile = segmenter->process(frontCamera->getImage());
        command = outbound.step(snapshot, localizer, segmenter ? &profile : nullptr,
                                sphereCamera ? sphereCamera->getImage() : nullptr, dtS);
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
