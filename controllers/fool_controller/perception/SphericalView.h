#ifndef RESCUE_SPHERICAL_VIEW_H_
#define RESCUE_SPHERICAL_VIEW_H_

#include <cmath>

// Geometría de la cámara esférica (camera_s) del Pioneer2, obtenida y validada
// con capturas reales de Webots (results/captures, 6 poses del escenario 01):
//
//  - Montaje: 0.44 m de altura, 0.05 m DELANTE y 0.025 m a la DERECHA del centro
//    del robot, mirando hacia abajo. En la imagen: arriba = delante del robot,
//    izquierda = izquierda del robot.
//  - Proyección: cilíndrica (equirectangular) alrededor del eje de la cámara:
//      longitud  lambda = (u + 0.5 - cx) / k   (positiva hacia la derecha)
//      latitud   phi    = -(v + 0.5 - cy) / k  (positiva hacia delante)
//    con k = 160/pi px/rad (180° en 160 px) y centro (80, 80). Dirección del rayo
//    en el marco de la cámara: (derecha, delante, abajo) =
//      (sin(lambda) cos(phi), sin(phi), cos(lambda) cos(phi)).
//    Un rayo que baja corta el suelo en
//      derecha = h * dr / dd,   delante = h * df / dd.
//    Este modelo reproduce el 99.3 % de los píxeles suelo/pared de las capturas.
//  - El propio robot ocupa el centro de la imagen (disco de ~40 px de radio,
//    algo desplazado hacia atrás): máscara medida en las capturas reales
//    (píxeles blancos constantes + marca amarilla + cruz, ampliados 3 px).
//  - Resolución en el suelo (aprox.): 5 cm/px a 1 m, 20 cm/px a 2 m, 80 cm/px a 4 m.
//    Zona ciega: no se ve el suelo a menos de ~0.45 m del robot.
//
// Implementación solo en cabecera para no tener que tocar el Makefile.
class SphericalView {
public:
  static constexpr int kWidth = 160;
  static constexpr int kHeight = 160;
  static constexpr double kFov = 3.14159265358979;   // 180°
  static constexpr double kCameraHeightM = 0.44;
  static constexpr double kCameraForwardM = 0.05;    // respecto al centro del robot
  static constexpr double kCameraRightM = 0.025;

  // ¿Coincide la cámara real con la geometría medida?
  static bool matches(int width, int height, double fov) {
    return width == kWidth && height == kHeight && std::fabs(fov - kFov) < 1e-3;
  }

  // ¿El píxel (u, v) muestra al propio robot?
  static bool isRobot(int u, int v) {
    if (v < 0 || v >= kHeight || u < 0 || u >= kWidth) return false;
    return maskXMin()[v] >= 0 && u >= maskXMin()[v] && u <= maskXMax()[v];
  }

  // Punto del suelo que ve el píxel, en metros desde el centro del robot
  // (forwardM hacia delante, leftM hacia la izquierda). false si el rayo no baja
  // (por encima del horizonte) o el punto está más lejos de maxRangeM.
  static bool pixelToGround(int u, int v, double maxRangeM, double &forwardM, double &leftM) {
    const double k = kWidth / kFov;
    const double lambda = (u + 0.5 - 0.5 * kWidth) / k;
    const double phi = -(v + 0.5 - 0.5 * kHeight) / k;
    const double dr = std::sin(lambda) * std::cos(phi);
    const double df = std::sin(phi);
    const double dd = std::cos(lambda) * std::cos(phi);
    if (dd < 1e-3) return false;
    const double right = kCameraHeightM * dr / dd + kCameraRightM;
    const double forward = kCameraHeightM * df / dd + kCameraForwardM;
    if (std::hypot(right, forward) > maxRangeM) return false;
    forwardM = forward;
    leftM = -right;
    return true;
  }

private:
  // Por fila: columnas mínima y máxima del robot (-1 = fila sin robot). Variables
  // estáticas dentro de funciones: así el archivo se puede incluir desde varios
  // .cpp sin definiciones duplicadas (C++11/14).
  static const int *maskXMin() {
    static const int values[160] = {
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 73, 70, 69,
    67, 63, 61, 58, 56, 54, 52, 51, 50, 49, 48, 48, 47, 46, 45, 44,
    43, 41, 40, 39, 38, 38, 37, 36, 36, 36, 35, 35, 34, 35, 35, 35,
    35, 35, 35, 35, 35, 35, 34, 33, 33, 33, 33, 33, 34, 35, 35, 35,
    35, 35, 35, 35, 35, 34, 35, 36, 37, 37, 37, 38, 39, 39, 39, 39,
    39, 39, 39, 39, 39, 39, 40, 39, 40, 41, 42, 43, 44, 45, 47, 48,
    50, 51, 54, 55, 59, 62, 67, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  };
    return values;
  }
  static const int *maskXMax() {
    static const int values[160] = {
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, 75, 76, 77,
    85, 88, 91, 93, 95, 97, 99, 100, 102, 103, 104, 105, 107, 108, 109, 110,
    111, 112, 113, 113, 114, 114, 115, 115, 116, 117, 117, 118, 117, 117, 117, 117,
    116, 117, 117, 118, 119, 119, 119, 119, 119, 119, 119, 119, 119, 119, 119, 119,
    118, 117, 117, 117, 117, 117, 117, 117, 116, 115, 114, 113, 114, 114, 114, 114,
    113, 112, 113, 114, 113, 114, 113, 112, 112, 113, 112, 111, 110, 109, 107, 106,
    105, 103, 98, 97, 96, 92, 87, -1, -1, -1, -1, -1, -1, -1, -1, -1,
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,
  };
    return values;
  }
};

#endif  // RESCUE_SPHERICAL_VIEW_H_
