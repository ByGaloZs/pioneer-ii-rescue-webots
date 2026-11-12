#ifndef RESCUE_FLOOR_SEGMENTER_H_
#define RESCUE_FLOOR_SEGMENTER_H_

#include <vector>

// Clase de cada píxel según su color.
enum class PixelClass { kFloor, kObstacle, kVictim };

// Punto del suelo donde termina el espacio libre en una columna de la imagen,
// expresado en el marco del robot (metros desde el centro del robot):
//   forwardM  hacia delante,  leftM  hacia la izquierda.
struct FreeSpacePoint {
  double forwardM;
  double leftM;
  bool closeObstacle;  // el obstáculo llega al borde inferior: está a menos de ~1 m
};

// Perfil de espacio libre visto por la cámara frontal.
struct FreeSpaceProfile {
  bool valid = false;
  std::vector<FreeSpacePoint> points;  // uno por columna procesada
  int victimPixels = 0;                // píxeles verdes (para la fase de víctimas)
  int floorBrightness = 0;             // brillo del suelo de referencia (0-255)

  // Distancia libre (m, desde el centro del robot) a lo largo de una dirección
  // relativa al robot (rad, positivo = izquierda) para un pasillo del ancho del
  // robot. Devuelve maxRangeM si no hay nada.
  double corridorClearance(double relativeHeadingRad, double halfWidthM, double maxRangeM) const;
};

// Segmenta la imagen de la cámara frontal en suelo / obstáculo (cian) /
// víctima (verde) y, recorriendo cada columna de abajo hacia el horizonte,
// obtiene dónde empieza el primer obstáculo. Supone suelo plano y cámara
// horizontal (montaje original del Pioneer2: altura 0.255 m, 0.155 m delante).
//
// Robusto a la iluminación de cada escenario:
//  - En cada imagen aprende el color del suelo con la franja inferior central
//    (mediana), y clasifica cada píxel RELATIVO a ese color (cociente g/r y
//    b/r), no con umbrales fijos.
//  - Si el suelo de referencia es demasiado oscuro, el perfil sale no válido y
//    el navegador funciona solo con infrarrojos.
//  - Píxeles casi negros (no se sabe qué son) y la zona lejana (niebla) se
//    tratan de forma conservadora: no cuentan como espacio libre.
class FloorSegmenter {
public:
  FloorSegmenter(int width, int height, double horizontalFovRad);

  // image: buffer BGRA de webots::Camera::getImage().
  FreeSpaceProfile process(const unsigned char *image) const;

  // Clasificación con umbrales fijos (respaldo, y para la fase de víctimas).
  static PixelClass classify(int red, int green, int blue);

private:
  double groundForwardM(int row) const;  // distancia al suelo visto en esa fila

  int _width;
  int _height;
  double _focalPx;
  int _columnStep;
};

#endif  // RESCUE_FLOOR_SEGMENTER_H_
