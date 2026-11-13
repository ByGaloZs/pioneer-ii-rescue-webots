#ifndef RESCUE_SENSOR_MANAGER_H_
#define RESCUE_SENSOR_MANAGER_H_

#include <array>
#include <deque>

#include <webots/Camera.hpp>
#include <webots/Compass.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/GPS.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Robot.hpp>

#include "SensorSnapshot.h"

// Inicializa GPS, brújula, encoders y los 16 sensores de distancia, y entrega
// en cada paso un SensorSnapshot con las lecturas ya convertidas.
// La cámara frontal se activa bajo demanda; la esférica, en la fase de víctimas.
class SensorManager {
public:
  explicit SensorManager(webots::Robot *robot);

  // Busca y activa todos los dispositivos; regresa false si falta alguno.
  bool initialize(int timeStep);
  void shutdown();
  // Lee todo en el paso actual y regresa las lecturas ya convertidas.
  SensorSnapshot read();

  // Imprime tipo y alcance de cada sensor de distancia según Webots.
  void printDistanceSensorProfiles() const;

  // Cámara frontal: se activa solo si se usa (renderizarla cuesta tiempo).
  webots::Camera *enableFrontCamera();
  webots::Camera *frontCamera() const { return _frontCamera; }
  // Cámara esférica (360°, mira hacia abajo): se usa para el mapa, la detección de víctimas y las capturas.
  webots::Camera *enableSphericalCamera();
  webots::Camera *sphericalCamera() const { return _sphericalCamera; }

private:
  // Mediana móvil de las últimas lecturas de un sensor (quita picos de ruido).
  double filteredResponse(int index, double rawResponse);

  webots::Robot *_robot;
  int _timeStep;
  webots::GPS *_gps;
  webots::Compass *_compass;
  webots::PositionSensor *_leftEncoder;
  webots::PositionSensor *_rightEncoder;
  std::array<webots::DistanceSensor *, robot_config::kDistanceSensorCount> _distanceSensors;
  // Historial de respuestas por sensor para la mediana.
  std::array<std::deque<double>, robot_config::kDistanceSensorCount> _responseHistory;
  webots::Camera *_frontCamera = nullptr;
  webots::Camera *_sphericalCamera = nullptr;
  bool _initialized;
};

#endif  // RESCUE_SENSOR_MANAGER_H_
