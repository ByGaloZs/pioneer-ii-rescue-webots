#ifndef RESCUE_SNAPSHOT_RECORDER_H_
#define RESCUE_SNAPSHOT_RECORDER_H_

#include <webots/Camera.hpp>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <dirent.h>
#include <sys/stat.h>

#include "../common/RunOutput.h"
#include "../localization/Localizer.h"

// Guarda imágenes de las cámaras junto con la pose del robot, para estudiar
// cómo las renderiza Webots (cámara esférica: orientación, proyección, zona que
// tapa el propio robot -> máscara) y para calibrar la cámara frontal.
//
// Escribe en results/captures/:
//   NNNN_front.png   cámara frontal
//   NNNN_sphere.png  cámara esférica
//   index.csv        id,t_s,x,z,yaw_deg y tamaños de imagen
// Solo guarda si ha pasado el periodo; con onlyIfMoved, además solo si el robot
// se ha movido (modo "capture": se arrastra el robot a distintos sitios).
// Implementación solo en cabecera para no tocar el Makefile.
class SnapshotRecorder {
public:
  SnapshotRecorder(webots::Camera *front, webots::Camera *sphere, double periodS,
                   bool onlyIfMoved)
      : _front(front), _sphere(sphere), _period(periodS), _onlyIfMoved(onlyIfMoved) {
    _dir = run_output::path("captures");
    ::mkdir(_dir.c_str(), 0755);
    clearOldCaptures();  // cada ejecución empieza de cero
    std::cout << "[CAPTURE] Capturas ACTIVADAS (cada " << _period << " s"
              << (_onlyIfMoved ? ", solo si el robot se mueve" : "") << ") en results/captures/"
              << std::endl;
  }

  void update(double timeS, const Pose &pose) {
    if (timeS < _next) return;
    if (_onlyIfMoved && _count > 0 && std::hypot(pose.xM - _lastX, pose.zM - _lastZ) < 0.03 &&
        std::fabs(pose.yawRad - _lastYaw) < 0.02) {
      _next = timeS + _period;
      return;
    }
    _next = timeS + _period;
    char id[16];
    std::snprintf(id, sizeof id, "%04d", _count);
    int failed = 0;
    if (_front != nullptr) failed += _front->saveImage(_dir + "/" + id + "_front.png", 100) != 0;
    if (_sphere != nullptr) failed += _sphere->saveImage(_dir + "/" + id + "_sphere.png", 100) != 0;
    if (failed != 0)
      std::cerr << "[CAPTURE] AVISO: Webots no pudo guardar " << failed << " imagen(es) " << id
                << std::endl;

    const std::string indexPath = _dir + "/index.csv";
    const bool exists = std::ifstream(indexPath).good();
    std::ofstream index(indexPath, std::ios::app);
    if (!exists) index << "id,t_s,x,z,yaw_deg,front_wh,sphere_wh,sphere_fov_rad\n";
    index << id << ',' << timeS << ',' << pose.xM << ',' << pose.zM << ','
          << pose.yawRad * 180.0 / 3.14159265358979 << ','
          << (_front ? std::to_string(_front->getWidth()) + "x" + std::to_string(_front->getHeight()) : "-")
          << ','
          << (_sphere ? std::to_string(_sphere->getWidth()) + "x" + std::to_string(_sphere->getHeight()) : "-")
          << ',' << (_sphere ? _sphere->getFov() : 0.0) << '\n';
    std::cout << "[CAPTURE] " << id << " guardada en results/captures/ pos=(" << pose.xM << ", "
              << pose.zM << ") yaw=" << pose.yawRad * 180.0 / 3.14159265358979 << "°" << std::endl;
    _lastX = pose.xM;
    _lastZ = pose.zM;
    _lastYaw = pose.yawRad;
    ++_count;
  }

private:
  void clearOldCaptures() const {
    if (DIR *dir = ::opendir(_dir.c_str())) {
      while (const dirent *entry = ::readdir(dir)) {
        const std::string name = entry->d_name;
        const bool image = name.size() > 4 && name.compare(name.size() - 4, 4, ".png") == 0;
        if (image || name == "index.csv") std::remove((_dir + "/" + name).c_str());
      }
      ::closedir(dir);
    }
  }

  webots::Camera *_front;
  webots::Camera *_sphere;
  double _period;
  bool _onlyIfMoved;
  std::string _dir;
  double _next = 0.0;
  double _lastX = 0.0, _lastZ = 0.0, _lastYaw = 0.0;
  int _count = 0;
};

#endif  // RESCUE_SNAPSHOT_RECORDER_H_
