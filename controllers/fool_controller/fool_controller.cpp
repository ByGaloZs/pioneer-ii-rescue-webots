// Controlador del Pioneer II para la misión de rescate.
//
// Modos (RobotConfig.h::kStartupMode o controllerArgs del robot en el .wbt):
//   mission    (por defecto) misión completa: ida, buscar y rescatar las 2 víctimas
//              (parar < 1 m, giro de 360°, mensaje) y volver a la salida.
//   calibrate  rutina que mide GPS, brújula, encoders e IR.
//
// Opciones de la misión (controllerArgs, una por línea):
//   nocam      sin cámara frontal          nomap     sin mapa + A* (Bug puro)
//   nosphere   sin cámara esférica         nolive    sin volcado del mapa en directo
//   nofiles    no escribir nada en disco (ni CSV, ni mapas, ni log): solo consola
//   timelimit=SEG  límite total de la misión en segundos (por defecto 240)
//   tag=XX     etiqueta de la prueba (si falta, se detecta el mundo)

#include <webots/Robot.hpp>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

#include "calibration/SensorCalibration.h"
#include "perception/SphericalView.h"
#include "common/ConsoleLog.h"
#include "common/RunOutput.h"
#include "config/RobotConfig.h"
#include "control/MotorController.h"
#include "localization/Localizer.h"
#include "mission/RescueMission.h"
#include "perception/FloorSegmenter.h"
#include "sensors/SensorManager.h"

namespace {

// Opciones de ejecución: modo y banderas que se leen de los controllerArgs.
struct Options {
  robot_config::StartupMode mode = robot_config::kStartupMode;
  bool useCamera = robot_config::kUseCameraByDefault;
  bool useMap = robot_config::kUseMapByDefault;
  bool liveMap = robot_config::kLiveMapByDefault;
  bool useSphere = robot_config::kUseSphereByDefault;
  bool files = true;  // "nofiles": no escribir CSV, mapas ni log
  double timeLimitS = robot_config::nav::kMissionTimeLimitS;  // "timelimit=SEG"
  std::string tag;
};

// Recorre los argumentos y activa o desactiva cada opción por su nombre.
Options parseOptions(int argc, char **argv) {
  Options options;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "calibrate") options.mode = robot_config::StartupMode::kCalibrate;
    else if (arg == "mission") options.mode = robot_config::StartupMode::kMission;
    else if (arg == "nofiles") options.files = false;
    else if (arg.rfind("timelimit=", 0) == 0) {
      const double seconds = std::atof(arg.c_str() + 10);
      if (seconds > 0.0) options.timeLimitS = seconds;
    }
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

}  // namespace

// Punto de entrada: arma los módulos según las opciones y corre el ciclo de control.
int main(int argc, char **argv) {
  webots::Robot robot;
  const int timeStep = static_cast<int>(robot.getBasicTimeStep());
  // Duración de un paso de simulación en segundos.
  const double dtS = timeStep / 1000.0;

  // Las opciones se leen primero para nombrar el log con el mundo.
  const Options options = parseOptions(argc, argv);
  run_output::setFilesEnabled(options.files);
  // Todo lo que se imprime en consola se copia también a logs/<mundo>_<fecha>.log
  // (salvo con "nofiles").
  std::unique_ptr<console_log::Session> log;
  if (options.files) {
    log.reset(new console_log::Session(run_output::resolveTag(options.tag)));
    if (log->active())
      std::cout << "[STARTUP] Consola guardada en " << log->path().substr(6)  // sin "../../"
                << std::endl;
    else
      std::cerr << "[STARTUP] No se pudo crear el archivo de log; solo consola." << std::endl;
  } else {
    std::cout << "[STARTUP] nofiles: no se guarda nada en disco (ni CSV, ni mapas, ni log)."
              << std::endl;
  }

  // Motores: se inicializan y se dejan detenidos.
  MotorController motors(&robot);
  motors.initialize();
  motors.stop();

  // Sensores: si falla alguno, no tiene caso seguir.
  SensorManager sensors(&robot);
  if (!sensors.initialize(timeStep)) {
    std::cerr << "[STARTUP] No se pudieron inicializar los sensores." << std::endl;
    return 1;
  }
  sensors.printDistanceSensorProfiles();

  std::cout << "[STARTUP] controllerArgs recibidos (" << argc - 1 << "):";
  for (int i = 1; i < argc; ++i) std::cout << " [" << argv[i] << "]";
  std::cout << std::endl;
  const bool mission = options.mode == robot_config::StartupMode::kMission;
  std::cout << "[STARTUP] Modo: "
            << (mission ? "misión completa (ida, víctimas, vuelta)" : "calibración") << std::endl;

  // Cámara frontal solo si se navega con ella.
  std::unique_ptr<FloorSegmenter> segmenter;
  webots::Camera *frontCamera = nullptr;
  if (mission && options.useCamera) {
    frontCamera = sensors.enableFrontCamera();
    if (frontCamera != nullptr)
      segmenter.reset(new FloorSegmenter(frontCamera->getWidth(), frontCamera->getHeight(),
                                         frontCamera->getFov()));
  }

  // Cámara esférica: mapa y detección de víctimas (solo en la misión).
  webots::Camera *sphereCamera = nullptr;
  if (mission && options.useSphere) {
    sphereCamera = sensors.enableSphericalCamera();
    if (sphereCamera != nullptr && !SphericalView::matches(sphereCamera->getWidth(),
                                                          sphereCamera->getHeight(),
                                                          sphereCamera->getFov())) {
      std::cerr << "[SENSORS] La cámara esférica no tiene la geometría medida (160x160, 180°); "
                   "no se usa para el mapa." << std::endl;
      sphereCamera = nullptr;
    }
  }
  if (mission) {
    // Estado de la cámara esférica al arrancar y, si no se usa, por qué.
    std::cout << "[STARTUP] Cámara esférica: ";
    if (sphereCamera != nullptr)
      std::cout << "EN USO (se ignora cada imagen con el suelo más oscuro de "
                << robot_config::mapping::kSphereMinBrightness << "/255)";
    else if (!options.useSphere)
      std::cout << "DESACTIVADA por 'nosphere' en controllerArgs";
    else
      std::cout << "DESACTIVADA: no se encontró o no tiene la geometría medida";
    std::cout << std::endl;
  }

  // Módulos principales: localización, calibración y misión completa. El mapa en
  // directo es un archivo: con "nofiles" no se vuelca.
  Localizer localizer;
  SensorCalibration calibration;
  RescueMission rescue(options.useCamera && segmenter != nullptr, options.useMap,
                       options.liveMap && options.files, sphereCamera != nullptr, options.tag);
  rescue.setTimeLimitS(options.timeLimitS);
  if (options.timeLimitS != robot_config::nav::kMissionTimeLimitS)
    std::cout << "[STARTUP] timelimit: límite de la misión " << options.timeLimitS
              << " s (el enunciado pide " << robot_config::nav::kMissionTimeLimitS << " s)"
              << std::endl;
  // Siguiente instante en el que se imprime el estado (una vez por segundo).
  double nextStatusS = 0.0;

  // Ciclo principal: leer sensores, estimar pose, decidir y mandar a los motores.
  while (robot.step(timeStep) != -1) {
    const SensorSnapshot snapshot = sensors.read();
    localizer.update(snapshot);

    WheelCommand command = WheelCommand::stop();
    switch (options.mode) {
      case robot_config::StartupMode::kCalibrate:
        if (!calibration.finished()) command = calibration.step(snapshot, localizer);
        break;
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
    }
    // Se envía a los motores el comando que decidió el modo activo.
    motors.setWheelSpeeds(command.leftRadPerSec, command.rightRadPerSec);
  }

  motors.stop();
  sensors.shutdown();
  return 0;
}
